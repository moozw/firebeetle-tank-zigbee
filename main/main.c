/*
 * FireBeetle 2 ESP32-C5  -  Zigbee tank-level fill controller.
 *
 *  - Tank sensor (0x5D) measures absolute pressure at the tank bottom.
 *  - Barometric reference sensor (0x5C) measures ambient pressure.
 *  - depth = (P_tank - P_baro) / (rho * g)   -> reported in cm and %.
 *  - Relay on D13/GPIO15 (Adafruit Power Relay FeatherWing) drives the fill
 *    pump: ON when depth <= operating setpoint, OFF when depth >= full setpoint.
 *  - Mains powered => Zigbee ROUTER (repeats the mesh for other devices).
 *  - Low/full setpoints, tank height, water density and an auto/on/off mode
 *    override are all writable live from Zigbee2MQTT.
 *
 * Build: ESP-IDF v5.5+, target esp32c5, with the espressif/esp-zigbee-lib
 * component (see idf_component.yml).
 */
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_zigbee_core.h"

#include "app_config.h"
#include "lps2x.h"

static const char *TAG = "tank";

/* ----------------------------- shared state ----------------------------- */
typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint8_t  calibrated;
    int16_t  low_cm;
    int16_t  operating_cm;
    int16_t  full_cm;
    int16_t  tank_h_cm;
    uint16_t density;
    uint8_t  mode;          /* MODE_AUTO / MODE_ON / MODE_OFF */
    uint8_t  conn_mode;     /* CONN_* */
    int16_t  air_offset_hpa_x100;
    char     wifi_ssid[33];
    char     wifi_pass[65];
    char     mqtt_host[65];
    char     mqtt_user[33];
    char     mqtt_pass[65];
    char     mqtt_topic[33];
} cfg_t;

static cfg_t g_cfg = {
    .magic     = 0x544b,     /* "TK" */
    .version   = 4,
    .calibrated = 0,
    .low_cm    = DEFAULT_LEVEL_LOW_CM,
    .operating_cm = DEFAULT_OPERATING_CM,
    .full_cm   = DEFAULT_LEVEL_FULL_CM,
    .tank_h_cm = DEFAULT_TANK_HEIGHT_CM,
    .density   = DEFAULT_WATER_DENSITY,
    .mode      = MODE_AUTO,
    .conn_mode = CONN_UNSET,
    .air_offset_hpa_x100 = 0,
    .wifi_ssid = "",
    .wifi_pass = "",
    .mqtt_host = DEFAULT_MQTT_HOST,
    .mqtt_user = "",
    .mqtt_pass = "",
    .mqtt_topic = DEFAULT_MQTT_TOPIC,
};
static SemaphoreHandle_t g_cfg_mtx;
static void cfg_save(void);

static const cfg_t DEFAULT_CFG = {
    .magic     = 0x544b,
    .version   = 4,
    .calibrated = 0,
    .low_cm    = DEFAULT_LEVEL_LOW_CM,
    .operating_cm = DEFAULT_OPERATING_CM,
    .full_cm   = DEFAULT_LEVEL_FULL_CM,
    .tank_h_cm = DEFAULT_TANK_HEIGHT_CM,
    .density   = DEFAULT_WATER_DENSITY,
    .mode      = MODE_AUTO,
    .conn_mode = CONN_UNSET,
    .air_offset_hpa_x100 = 0,
    .wifi_ssid = "",
    .wifi_pass = "",
    .mqtt_host = DEFAULT_MQTT_HOST,
    .mqtt_user = "",
    .mqtt_pass = "",
    .mqtt_topic = DEFAULT_MQTT_TOPIC,
};

/* live telemetry (updated by control task) */
static int16_t g_depth_cm = 0;
static int16_t g_baro_pressure_hpa = -1;
static int16_t g_tank_pressure_hpa = -1;
static int32_t g_baro_pressure_hpa_x100 = -1;
static int32_t g_tank_pressure_hpa_x100 = -1;
static uint8_t g_level_pct = 0;
static uint8_t g_fault     = 0;
static uint8_t g_low_alert = 0;
static bool    g_relay_on  = false;
static volatile bool g_zb_joined = false;   /* true once on a Zigbee network */
static volatile int64_t g_join_us = 0;      /* timestamp (us) when we joined   */
static volatile bool s_ota_active = false;  /* true while an OTA is downloading */
static volatile bool s_wifi_connected = false;
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;
static char s_mqtt_uri[96];
static httpd_handle_t s_httpd = NULL;
static int s_wifi_retry = 0;

/* ----------------------------- NVS persistence -------------------------- */
static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open("tank", NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = sizeof(g_cfg);
    esp_err_t err = nvs_get_blob(h, "cfg", &g_cfg, &sz);
    nvs_close(h);

    bool bad = err != ESP_OK
        || sz != sizeof(g_cfg)
        || g_cfg.magic != DEFAULT_CFG.magic
        || g_cfg.version != DEFAULT_CFG.version
        || g_cfg.calibrated > 1
        || g_cfg.low_cm < 0
        || g_cfg.operating_cm < 0
        || g_cfg.operating_cm < g_cfg.low_cm
        || g_cfg.full_cm <= g_cfg.operating_cm
        || g_cfg.tank_h_cm < 10
        || g_cfg.tank_h_cm > 1000
        || g_cfg.full_cm > g_cfg.tank_h_cm + MAX_DEPTH_OVER_TANK_CM
        || g_cfg.density < 900
        || g_cfg.density > 1100
        || g_cfg.mode > MODE_OFF
        || (g_cfg.conn_mode != CONN_UNSET && g_cfg.conn_mode > CONN_WIFI)
        || g_cfg.wifi_ssid[sizeof(g_cfg.wifi_ssid) - 1] != '\0'
        || g_cfg.wifi_pass[sizeof(g_cfg.wifi_pass) - 1] != '\0'
        || g_cfg.mqtt_host[sizeof(g_cfg.mqtt_host) - 1] != '\0'
        || g_cfg.mqtt_user[sizeof(g_cfg.mqtt_user) - 1] != '\0'
        || g_cfg.mqtt_pass[sizeof(g_cfg.mqtt_pass) - 1] != '\0'
        || g_cfg.mqtt_topic[sizeof(g_cfg.mqtt_topic) - 1] != '\0';
    if (bad) {
        ESP_LOGW(TAG, "stored config invalid, restoring defaults");
        g_cfg = DEFAULT_CFG;
        cfg_save();
    }
}

static void cfg_save(void)
{
    nvs_handle_t h;
    if (nvs_open("tank", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_blob(h, "cfg", &g_cfg, sizeof(g_cfg));
    nvs_commit(h);
    nvs_close(h);
}

static void copy_form_str(const char *form, const char *key, char *dst, size_t dst_len)
{
    char val[96];
    if (dst_len == 0) return;
    if (httpd_query_key_value(form, key, val, sizeof(val)) == ESP_OK) {
        strlcpy(dst, val, dst_len);
    }
}

static int form_int(const char *form, const char *key, int fallback)
{
    char val[16];
    if (httpd_query_key_value(form, key, val, sizeof(val)) == ESP_OK) {
        return atoi(val);
    }
    return fallback;
}

static void mqtt_publish_state(void)
{
    if (!s_mqtt || !s_mqtt_connected) return;

    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char topic[64];
    char payload[384];
    snprintf(topic, sizeof(topic), "%s/state", cfg.mqtt_topic[0] ? cfg.mqtt_topic : DEFAULT_MQTT_TOPIC);
    int n = snprintf(payload, sizeof(payload),
        "{\"depth_cm\":%d,\"level_pct\":%u,\"fault\":%u,\"low_alert\":%u,"
        "\"relay\":%s,\"mode\":%u,\"low_cm\":%d,\"operating_cm\":%d,\"full_cm\":%d,"
        "\"baro_hpa\":%.2f,\"tank_hpa\":%.2f}",
        g_depth_cm, g_level_pct, g_fault, g_low_alert, g_relay_on ? "true" : "false",
        cfg.mode, cfg.low_cm, cfg.operating_cm, cfg.full_cm,
        g_baro_pressure_hpa_x100 >= 0 ? (double)g_baro_pressure_hpa_x100 / 100.0 : -1.0,
        g_tank_pressure_hpa_x100 >= 0 ? (double)g_tank_pressure_hpa_x100 / 100.0 : -1.0);
    esp_mqtt_client_publish(s_mqtt, topic, payload, n, 0, 0);
}

static bool json_has(const char *json, const char *key, char *val, size_t val_len)
{
    char pat[32];
    snprintf(pat, sizeof(pat), "\"%s\"", key);
    char *p = strstr(json, pat);
    if (!p) return false;
    p = strchr(p, ':');
    if (!p) return false;
    p++;
    while (*p == ' ' || *p == '\"') p++;
    size_t i = 0;
    while (*p && *p != ',' && *p != '}' && *p != '\"' && i + 1 < val_len) {
        val[i++] = *p++;
    }
    val[i] = '\0';
    return i > 0;
}

static void mqtt_apply_command(const char *payload, int len)
{
    char json[256];
    size_t n = len < (int)sizeof(json) - 1 ? (size_t)len : sizeof(json) - 1;
    memcpy(json, payload, n);
    json[n] = '\0';

    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char val[24];
    if (json_has(json, "mode", val, sizeof(val))) {
        if (strcmp(val, "auto") == 0) cfg.mode = MODE_AUTO;
        else if (strcmp(val, "force_on") == 0 || strcmp(val, "on") == 0) cfg.mode = MODE_ON;
        else if (strcmp(val, "force_off") == 0 || strcmp(val, "off") == 0) cfg.mode = MODE_OFF;
        else cfg.mode = (uint8_t)atoi(val);
    }
    if (json_has(json, "low_cm", val, sizeof(val))) cfg.low_cm = atoi(val);
    if (json_has(json, "operating_cm", val, sizeof(val))) cfg.operating_cm = atoi(val);
    if (json_has(json, "full_cm", val, sizeof(val))) cfg.full_cm = atoi(val);

    if (cfg.low_cm < 0 || cfg.operating_cm < cfg.low_cm || cfg.full_cm <= cfg.operating_cm ||
        cfg.mode > MODE_OFF) {
        ESP_LOGW(TAG, "MQTT command rejected: %s", json);
        return;
    }

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg.low_cm = cfg.low_cm;
    g_cfg.operating_cm = cfg.operating_cm;
    g_cfg.full_cm = cfg.full_cm;
    g_cfg.mode = cfg.mode;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    ESP_LOGI(TAG, "MQTT cfg: lowAlert=%d operating=%d full=%d mode=%u",
             cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.mode);
    mqtt_publish_state();
}

/* ----------------------------- WiFi OTA --------------------------------- */
/* Triggered by an MQTT message on <topic>/ota: {"url":"http://host/fw.bin"}.
 * Uses esp_https_ota (handles partition/erase/write/verify/set-boot) - reuses
 * the same ota_0/ota_1 slots as Zigbee OTA. Status -> <topic>/ota_status. */
static char s_ota_url[256];

static void mqtt_ota_publish(const char *json_status)
{
    if (!s_mqtt || !s_mqtt_connected) return;
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);
    char topic[64];
    snprintf(topic, sizeof(topic), "%s/ota_status", cfg.mqtt_topic[0] ? cfg.mqtt_topic : DEFAULT_MQTT_TOPIC);
    esp_mqtt_client_publish(s_mqtt, topic, json_status, 0, 1, 0);
}

static void wifi_ota_task(void *arg)
{
    ESP_LOGI(TAG, "WiFi OTA from %s", s_ota_url);
    mqtt_ota_publish("{\"state\":\"downloading\"}");
    esp_http_client_config_t http = {
        .url = s_ota_url,
        .timeout_ms = 30000,
        .keep_alive_enable = true,
    };
    esp_https_ota_config_t ota_cfg = { .http_config = &http };
    esp_err_t err = esp_https_ota(&ota_cfg);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi OTA complete - rebooting");
        mqtt_ota_publish("{\"state\":\"success\"}");
        vTaskDelay(pdMS_TO_TICKS(1500));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "WiFi OTA failed: %s", esp_err_to_name(err));
        char msg[112];
        snprintf(msg, sizeof(msg), "{\"state\":\"failed\",\"error\":\"%s\"}", esp_err_to_name(err));
        mqtt_ota_publish(msg);
        s_ota_active = false;
    }
    vTaskDelete(NULL);
}

static void mqtt_ota_trigger(const char *payload, int len)
{
    if (s_ota_active) { mqtt_ota_publish("{\"state\":\"busy\"}"); return; }
    char json[320];
    size_t n = len < (int)sizeof(json) - 1 ? (size_t)len : sizeof(json) - 1;
    memcpy(json, payload, n);
    json[n] = '\0';

    char url[256];
    if (!json_has(json, "url", url, sizeof(url)) ||
        (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0)) {
        mqtt_ota_publish("{\"state\":\"failed\",\"error\":\"missing or invalid url\"}");
        return;
    }
    strlcpy(s_ota_url, url, sizeof(s_ota_url));
    s_ota_active = true;     /* pauses Zigbee telemetry during the download */
    if (xTaskCreate(wifi_ota_task, "wifi_ota", 8192, NULL, 5, NULL) != pdPASS) {
        s_ota_active = false;
        mqtt_ota_publish("{\"state\":\"failed\",\"error\":\"task create\"}");
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    if (event_id == MQTT_EVENT_CONNECTED) {
        s_mqtt_connected = true;
        cfg_t cfg;
        xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
        cfg = g_cfg;
        xSemaphoreGive(g_cfg_mtx);
        const char *base_topic = cfg.mqtt_topic[0] ? cfg.mqtt_topic : DEFAULT_MQTT_TOPIC;
        char topic[64];
        snprintf(topic, sizeof(topic), "%s/set", base_topic);
        esp_mqtt_client_subscribe(s_mqtt, topic, 0);
        snprintf(topic, sizeof(topic), "%s/ota", base_topic);
        esp_mqtt_client_subscribe(s_mqtt, topic, 0);
        ESP_LOGI(TAG, "MQTT connected, subscribed %s/{set,ota}", base_topic);
        mqtt_publish_state();
    } else if (event_id == MQTT_EVENT_DISCONNECTED) {
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "MQTT disconnected");
    } else if (event_id == MQTT_EVENT_DATA) {
        /* route by topic suffix: .../ota -> firmware update, else config command */
        if (event->topic && event->topic_len >= 4 &&
            strncmp(event->topic + event->topic_len - 4, "/ota", 4) == 0) {
            mqtt_ota_trigger(event->data, event->data_len);
        } else {
            mqtt_apply_command(event->data, event->data_len);
        }
    }
}

static void mqtt_start(void)
{
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);
    if (!cfg.mqtt_host[0]) {
        ESP_LOGI(TAG, "MQTT disabled: no broker host configured");
        return;
    }

    snprintf(s_mqtt_uri, sizeof(s_mqtt_uri), "mqtt://%s", cfg.mqtt_host);
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = s_mqtt_uri,
        .credentials.username = cfg.mqtt_user[0] ? cfg.mqtt_user : NULL,
        .credentials.authentication.password = cfg.mqtt_pass[0] ? cfg.mqtt_pass : NULL,
    };
    s_mqtt = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_mqtt) {
        ESP_LOGE(TAG, "MQTT init failed");
        return;
    }
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
    ESP_LOGI(TAG, "MQTT starting: %s topic=%s", s_mqtt_uri, cfg.mqtt_topic);
}

/* ----------------------------- relay ------------------------------------ */
static void relay_set(bool on)
{
    int level = on ? RELAY_ACTIVE_HIGH : !RELAY_ACTIVE_HIGH;
    gpio_set_level(RELAY_GPIO, level);
    g_relay_on = on;
}

static void relay_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RELAY_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    relay_set(false);          /* failsafe: pump OFF at boot */
}

/* -------------------------- Zigbee attribute push ----------------------- */
static inline bool zb_ready(void)
{
    /* joined, settled, and NOT mid-OTA (concurrent reports exhaust zboss buffers) */
    return g_zb_joined && !s_ota_active
        && (esp_timer_get_time() - g_join_us) >= TELEMETRY_START_DELAY_US;
}

static void zb_push_telemetry(void)
{
    if (!zb_ready()) return;
    esp_zb_lock_acquire(portMAX_DELAY);

    /* Just update attribute values. The stack auto-reports any attribute the
     * coordinator has configured for reporting; we do NOT send manual report
     * commands - those assert in zcl_general_commands if issued too early. */
    esp_err_t err = esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_DEPTH_CM, &g_depth_cm, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LEVEL_PCT, &g_level_pct, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_FAULT, &g_fault, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LOW_ALERT, &g_low_alert, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_BARO_PRESSURE_HPA, &g_baro_pressure_hpa, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TANK_PRESSURE_HPA, &g_tank_pressure_hpa, false);

    uint8_t on = g_relay_on ? 1 : 0;
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
        &g_baro_pressure_hpa, false);
    err |= esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID,
        &g_tank_pressure_hpa, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee telemetry attr update failed: %s", esp_err_to_name(err));
    }

    esp_zb_lock_release();
}

/* push the current mode to Zigbee so the hub reflects a local button change */
static void zb_push_mode(uint8_t mode)
{
    if (!zb_ready()) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_MODE, &mode, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee mode attr update failed: %s", esp_err_to_name(err));
    }
    esp_zb_lock_release();
}

/* ----------------------------- setup portal ----------------------------- */
#if SETUP_PORTAL_ENABLE
static esp_err_t setup_status_get(httpd_req_t *req)
{
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char json[980];
    int n = snprintf(json, sizeof(json),
        "{\"baro_hpa\":%.2f,\"tank_hpa\":%.2f,\"depth_cm\":%d,\"level_pct\":%u,"
        "\"fault\":%u,\"low_alert\":%u,\"relay\":%s,\"calibrated\":%u,\"air_offset_hpa\":%.2f,"
        "\"low_cm\":%d,\"operating_cm\":%d,\"full_cm\":%d,\"tank_height_cm\":%d,\"density\":%u,"
        "\"mode\":%u,\"conn_mode\":%u,\"wifi_connected\":%s,\"mqtt_connected\":%s,"
        "\"wifi_ssid\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_user\":\"%s\",\"mqtt_topic\":\"%s\"}",
        g_baro_pressure_hpa_x100 >= 0 ? (double)g_baro_pressure_hpa_x100 / 100.0 : -1.0,
        g_tank_pressure_hpa_x100 >= 0 ? (double)g_tank_pressure_hpa_x100 / 100.0 : -1.0,
        g_depth_cm, g_level_pct, g_fault, g_low_alert, g_relay_on ? "true" : "false",
        cfg.calibrated, (double)cfg.air_offset_hpa_x100 / 100.0,
        cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.tank_h_cm, cfg.density, cfg.mode, cfg.conn_mode,
        s_wifi_connected ? "true" : "false", s_mqtt_connected ? "true" : "false",
        cfg.wifi_ssid, cfg.mqtt_host, cfg.mqtt_user, cfg.mqtt_topic);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

static esp_err_t setup_root_get(httpd_req_t *req)
{
    static const char html[] =
        "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Tank Setup</title><style>"
        "body{font-family:system-ui,Segoe UI,sans-serif;margin:0;background:#f5f7f8;color:#172026}"
        "main{max-width:760px;margin:auto;padding:18px}section{background:white;border:1px solid #d8e0e4;border-radius:8px;padding:14px;margin:12px 0}"
        "h1{font-size:24px;margin:4px 0 12px}h2{font-size:17px;margin:0 0 10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
        ".v{padding:10px;background:#eef3f5;border-radius:6px}label{display:block;font-size:13px;margin:10px 0 3px}input,select,button{font:inherit;padding:10px;border-radius:6px;border:1px solid #b8c4ca;box-sizing:border-box;width:100%;color:#172026;background:#fff}"
        "button{background:#0b6f85;color:white;border:0;margin-top:10px}.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}.warn{color:#9a4b00}.ok{color:#157347}"
        "</style></head><body><main><h1>Tank Controller Setup</h1>"
        "<section><h2>Live Sensor Check</h2><div class=grid>"
        "<div class=v>Baro <b id=baro>-</b> hPa</div><div class=v>Tank <b id=tank>-</b> hPa</div>"
        "<div class=v>Depth <b id=depth>-</b> cm</div><div class=v>Low alert <b id=lowalert>-</b></div>"
        "<div class=v>Relay <b id=relay>-</b></div><div class=v>Fault <b id=fault>-</b></div>"
        "<div class=v>WiFi <b id=wifistat>-</b></div><div class=v>MQTT <b id=mqttstat>-</b></div>"
        "</div><p id=cal></p></section>"
        "<section><h2>1. Open-Air Test</h2><p>Keep both sensors in open air, then save the offset.</p><button onclick=\"post('/api/open_air')\">Run open-air test</button></section>"
        "<section><h2>2. Full Tank Calibration</h2><p>Lower the tank sensor into a full tank, keep the baro sensor in air, then save full.</p><button onclick=\"post('/api/full_tank')\">Set full tank</button></section>"
        "<section><h2>Operating Parameters</h2><div class=row>"
        "<label>Low alert cm<input id=low type=number></label><label>Operating level cm<input id=op type=number></label>"
        "<label>Full level cm<input id=full type=number></label>"
        "<label>Tank height cm<input id=height type=number></label><label>Density kg/m3<input id=density type=number></label>"
        "<label>Mode<select id=mode><option value=0>auto</option><option value=1>force on</option><option value=2>force off</option></select></label>"
        "<label>Connectivity<select id=conn><option value=255>choose</option><option value=0>AP</option><option value=1>zigbee</option><option value=2>local wifi</option></select></label>"
        "</div></section><section><h2>Local WiFi / MQTT</h2><div class=row>"
        "<label>WiFi SSID<input id=ssid></label><label>WiFi password<input id=wpass type=password autocomplete=off></label>"
        "<label>MQTT broker IP/host<input id=mh placeholder='192.168.1.10'></label><label>MQTT topic<input id=mt></label>"
        "<label>MQTT user<input id=mu></label><label>MQTT password<input id=mp type=password autocomplete=off></label>"
        "</div><button onclick=save()>Save parameters</button></section><p id=msg></p></main><script>"
        "const $=id=>document.getElementById(id);"
        "let editing=false;"
        "function bindEdits(){['low','op','full','height','density','mode','conn','ssid','wpass','mh','mu','mp','mt'].forEach(id=>{$(id).oninput=()=>editing=true;$(id).onchange=()=>editing=true})}"
        "function syncForm(s){$('low').value=s.low_cm;$('op').value=s.operating_cm;$('full').value=s.full_cm;$('height').value=s.tank_height_cm;$('density').value=s.density;$('mode').value=s.mode;$('conn').value=s.conn_mode;$('ssid').value=s.wifi_ssid;$('mh').value=s.mqtt_host;$('mu').value=s.mqtt_user;$('mt').value=s.mqtt_topic;if(!$('wpass').value)$('wpass').placeholder='saved/blank';if(!$('mp').value)$('mp').placeholder='saved/blank'}"
        "async function load(){let r=await fetch('/api/status');let s=await r.json();"
        "$('baro').textContent=s.baro_hpa.toFixed(2);$('tank').textContent=s.tank_hpa.toFixed(2);$('depth').textContent=s.depth_cm;$('fault').textContent=s.fault;"
        "$('lowalert').textContent=s.low_alert?'ON':'OFF';$('relay').textContent=s.relay?'ON':'OFF';"
        "$('wifistat').textContent=s.wifi_connected?'connected':'off';$('mqttstat').textContent=s.mqtt_connected?'connected':'off';"
        "$('cal').innerHTML=s.calibrated?'<span class=ok>Calibrated</span>':'<span class=warn>Not calibrated: AUTO pump control stays safe/off</span>';"
        "if(!editing)syncForm(s)}"
        "async function post(u){let r=await fetch(u,{method:'POST'});$('msg').textContent=await r.text();editing=false;load()}"
        "async function save(){let q=new URLSearchParams({low:$('low').value,op:$('op').value,full:$('full').value,height:$('height').value,density:$('density').value,mode:$('mode').value,conn:$('conn').value,ssid:$('ssid').value,wpass:$('wpass').value,mh:$('mh').value,mu:$('mu').value,mp:$('mp').value,mt:$('mt').value});let r=await fetch('/api/config',{method:'POST',body:q});$('msg').textContent=await r.text();if(r.ok){editing=false;$('wpass').value='';$('mp').value=''}load()}"
        "bindEdits();load();setInterval(load,3000)</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

static bool setup_has_two_pressures(void)
{
    return g_baro_pressure_hpa_x100 >= 0 && g_tank_pressure_hpa_x100 >= 0;
}

static esp_err_t setup_open_air_post(httpd_req_t *req)
{
    if (!setup_has_two_pressures()) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Both sensors must be reading in open air");
    }
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg.air_offset_hpa_x100 = (int16_t)(g_tank_pressure_hpa_x100 - g_baro_pressure_hpa_x100);
    g_cfg.calibrated = 0;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    return httpd_resp_sendstr(req, "Open-air offset saved. Now lower the tank sensor into a full tank.");
}

static esp_err_t setup_full_tank_post(httpd_req_t *req)
{
    if (!setup_has_two_pressures()) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Both sensors must be reading for full-tank calibration");
    }
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    float delta_hpa = ((float)(g_tank_pressure_hpa_x100 - g_baro_pressure_hpa_x100 - cfg.air_offset_hpa_x100)) / 100.0f;
    float depth_cm_f = (delta_hpa * 100.0f) / ((float)cfg.density * GRAVITY_MS2) * 100.0f + SENSOR_OFFSET_CM;
    int16_t full_cm = (int16_t)lroundf(depth_cm_f);
    if (full_cm < 10 || full_cm > 1000) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Full-tank pressure does not produce a plausible depth");
    }

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg.tank_h_cm = full_cm;
    g_cfg.full_cm = full_cm;
    g_cfg.low_cm = full_cm * 30 / 100;
    if (g_cfg.low_cm < 1) g_cfg.low_cm = 1;
    g_cfg.operating_cm = full_cm * 40 / 100;
    if (g_cfg.operating_cm <= g_cfg.low_cm) g_cfg.operating_cm = g_cfg.low_cm + 1;
    g_cfg.calibrated = 1;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    return httpd_resp_sendstr(req, "Full-tank calibration saved. Standalone AUTO control can now use the saved limits.");
}

static esp_err_t setup_config_post(httpd_req_t *req)
{
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg_t cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char form[900] = "";
    if (req->content_len > 0) {
        int got = httpd_req_recv(req, form, sizeof(form) - 1);
        if (got <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing form body");
        form[got] = '\0';
    } else if (httpd_req_get_url_query_str(req, form, sizeof(form)) != ESP_OK) {
        form[0] = '\0';
    }

    cfg.low_cm = form_int(form, "low", cfg.low_cm);
    cfg.operating_cm = form_int(form, "op", cfg.operating_cm);
    cfg.full_cm = form_int(form, "full", cfg.full_cm);
    cfg.tank_h_cm = form_int(form, "height", cfg.tank_h_cm);
    cfg.density = form_int(form, "density", cfg.density);
    cfg.mode = form_int(form, "mode", cfg.mode);
    cfg.conn_mode = form_int(form, "conn", cfg.conn_mode);
    copy_form_str(form, "ssid", cfg.wifi_ssid, sizeof(cfg.wifi_ssid));
    copy_form_str(form, "mh", cfg.mqtt_host, sizeof(cfg.mqtt_host));
    copy_form_str(form, "mu", cfg.mqtt_user, sizeof(cfg.mqtt_user));
    copy_form_str(form, "mt", cfg.mqtt_topic, sizeof(cfg.mqtt_topic));
    char maybe_pass[65] = "";
    copy_form_str(form, "wpass", maybe_pass, sizeof(maybe_pass));
    if (maybe_pass[0]) strlcpy(cfg.wifi_pass, maybe_pass, sizeof(cfg.wifi_pass));
    maybe_pass[0] = '\0';
    copy_form_str(form, "mp", maybe_pass, sizeof(maybe_pass));
    if (maybe_pass[0]) strlcpy(cfg.mqtt_pass, maybe_pass, sizeof(cfg.mqtt_pass));
    if (!cfg.mqtt_topic[0]) strlcpy(cfg.mqtt_topic, DEFAULT_MQTT_TOPIC, sizeof(cfg.mqtt_topic));

    if (cfg.low_cm < 0 || cfg.operating_cm < 0 || cfg.operating_cm < cfg.low_cm ||
        cfg.full_cm <= cfg.operating_cm ||
        cfg.tank_h_cm < 10 ||
        cfg.full_cm > cfg.tank_h_cm + MAX_DEPTH_OVER_TANK_CM || cfg.density < 900 ||
        cfg.density > 1100 || cfg.mode > MODE_OFF ||
        (cfg.conn_mode != CONN_UNSET && cfg.conn_mode > CONN_WIFI) ||
        (cfg.conn_mode == CONN_WIFI && !cfg.wifi_ssid[0])) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameter range");
    }

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg = cfg;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    zb_push_mode(cfg.mode);
    ESP_LOGI(TAG, "setup save: lowAlert=%d operating=%d full=%d conn=%u mode=%u",
             cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.conn_mode, cfg.mode);
    return httpd_resp_sendstr(req, "Parameters saved");
}

static void web_server_start(void)
{
    if (s_httpd) return;
    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.lru_purge_enable = true;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &http_cfg));
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/", .method=HTTP_GET, .handler=setup_root_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/status", .method=HTTP_GET, .handler=setup_status_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/open_air", .method=HTTP_POST, .handler=setup_open_air_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/full_tank", .method=HTTP_POST, .handler=setup_full_tank_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/config", .method=HTTP_POST, .handler=setup_config_post });
}

static void setup_portal_start(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    wifi_config_t ap_cfg = {0};
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s-%02X%02X", SETUP_AP_SSID_PREFIX, mac[4], mac[5]);
    ap_cfg.ap.ssid_len = strlen((char *)ap_cfg.ap.ssid);
    strlcpy((char *)ap_cfg.ap.password, SETUP_AP_PASSWORD, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.channel = SETUP_AP_CHANNEL;
    ap_cfg.ap.max_connection = SETUP_AP_MAX_CONN;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    web_server_start();

    ESP_LOGI(TAG, "setup AP started: SSID=%s URL=http://192.168.4.1", ap_cfg.ap.ssid);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        if (s_wifi_retry++ < WIFI_MAX_RETRY) {
            ESP_LOGW(TAG, "WiFi disconnected, retry %d/%d", s_wifi_retry, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi connect failed");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_wifi_retry = 0;
        s_wifi_connected = true;
        ESP_LOGI(TAG, "WiFi connected: http://" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static bool wifi_station_start(void)
{
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);
    if (!cfg.wifi_ssid[0]) {
        ESP_LOGE(TAG, "WiFi mode selected but no SSID is configured");
        return false;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t sta_cfg = {0};
    strlcpy((char *)sta_cfg.sta.ssid, cfg.wifi_ssid, sizeof(sta_cfg.sta.ssid));
    strlcpy((char *)sta_cfg.sta.password, cfg.wifi_pass, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    if (!cfg.wifi_pass[0]) sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    s_wifi_retry = 0;
    s_wifi_connected = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi connecting to SSID=%s", cfg.wifi_ssid);

    int64_t deadline = esp_timer_get_time() + (int64_t)WIFI_CONNECT_TIMEOUT_MS * 1000;
    while (!s_wifi_connected && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!s_wifi_connected) {
        ESP_LOGE(TAG, "WiFi connection timed out; falling back to setup AP");
        esp_wifi_stop();
        esp_wifi_deinit();
        return false;
    }

    web_server_start();
    mqtt_start();
    return true;
}

static bool setup_boot_button_held(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int64_t deadline = esp_timer_get_time() + (int64_t)SETUP_BOOT_WINDOW_MS * 1000;
    int64_t down_since = 0;
    while (esp_timer_get_time() < deadline) {
        bool down = gpio_get_level(BUTTON_GPIO) == 0;
        if (down) {
            if (down_since == 0) {
                down_since = esp_timer_get_time();
            } else if (esp_timer_get_time() - down_since >= (int64_t)SETUP_BOOT_HOLD_MS * 1000) {
                ESP_LOGW(TAG, "BOOT held during startup -> setup AP mode");
                return true;
            }
        } else {
            down_since = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}
#endif

/* ----------------------------- button task ------------------------------ */
/* short press: cycle AUTO -> FORCE_ON -> FORCE_OFF -> AUTO
 * long press (>= BUTTON_LONGPRESS_MS): Zigbee factory reset                  */
static void button_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    bool prev_down = false;
    int64_t press_us = 0;
    bool long_fired = false;

    while (1) {
        bool down = (gpio_get_level(BUTTON_GPIO) == 0);   /* active low */
        int64_t now = esp_timer_get_time();

        if (down && !prev_down) {
            press_us = now;
            long_fired = false;
        } else if (down && !long_fired &&
                   (now - press_us) >= (int64_t)BUTTON_LONGPRESS_MS * 1000) {
            ESP_LOGW(TAG, "long press -> Zigbee factory reset");
            long_fired = true;
            relay_set(false);
            esp_zb_factory_reset();          /* erases network, reboots */
        } else if (!down && prev_down && !long_fired) {
            if ((now - press_us) >= (int64_t)BUTTON_DEBOUNCE_MS * 1000) {
                uint8_t m;
                xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
                g_cfg.mode = (g_cfg.mode + 1) % 3;   /* AUTO/ON/OFF */
                m = g_cfg.mode;
                xSemaphoreGive(g_cfg_mtx);
                cfg_save();
                zb_push_mode(m);
                ESP_LOGI(TAG, "button -> mode %u (0=auto 1=on 2=off)", m);
            }
        }

        prev_down = down;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------------------------- control task ------------------------------ */
static void control_task(void *arg)
{
    i2c_master_bus_handle_t bus;
    lps2x_t baro = {0}, tank = {0};

    ESP_ERROR_CHECK(lps2x_bus_init(&bus));

    /* one-shot I2C scan to help confirm wiring / actual sensor addresses */
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d ...", I2C_SDA_GPIO, I2C_SCL_GPIO);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(bus, a, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  found I2C device @ 0x%02x", a);
            found++;
        }
    }
    if (found == 0) ESP_LOGW(TAG, "  no I2C devices found (sensors connected/powered?)");

    bool baro_ok = (lps2x_init(bus, LPS_BARO_ADDR, &baro) == ESP_OK);
    bool tank_ok = (lps2x_init(bus, LPS_TANK_ADDR, &tank) == ESP_OK);
    ESP_LOGI(TAG, "sensors: baro(0x%02x)=%s  tank(0x%02x)=%s", LPS_BARO_ADDR,
             baro_ok ? "OK" : "absent", LPS_TANK_ADDR, tank_ok ? "OK" : "absent");
    if (!baro_ok && !tank_ok) ESP_LOGW(TAG, "no sensors - relay stays in failsafe OFF");
    else if (!baro_ok || !tank_ok) ESP_LOGW(TAG, "single sensor only - level not measurable (diagnostic)");

    int fault_count = 0;
    int64_t last_off_us = 0, last_on_us = 0;

    while (1) {
        cfg_t cfg;
        xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
        cfg = g_cfg;
        xSemaphoreGive(g_cfg_mtx);

        /* read whichever sensors are present (each independent), averaging a
         * few slow one-shot conversions to calm long wires and startup noise. */
        float p_tank = 0, p_baro = 0;
        int baro_n = 0, tank_n = 0;
        for (int i = 0; i < SENSOR_AVG_SAMPLES; i++) {
            float p = 0;
            if (baro_ok && lps2x_read(&baro, &p, NULL) == ESP_OK) {
                p_baro += p;
                baro_n++;
            }
            if (tank_ok && lps2x_read(&tank, &p, NULL) == ESP_OK) {
                p_tank += p;
                tank_n++;
            }
            if (i + 1 < SENSOR_AVG_SAMPLES) vTaskDelay(pdMS_TO_TICKS(SENSOR_AVG_DELAY_MS));
        }
        bool baro_rd = baro_n > 0;
        bool tank_rd = tank_n > 0;
        if (baro_rd) p_baro /= (float)baro_n;
        if (tank_rd) p_tank /= (float)tank_n;
        g_baro_pressure_hpa = baro_rd ? (int16_t)lroundf(p_baro) : -1;
        g_tank_pressure_hpa = tank_rd ? (int16_t)lroundf(p_tank) : -1;
        g_baro_pressure_hpa_x100 = baro_rd ? (int32_t)lroundf(p_baro * 100.0f) : -1;
        g_tank_pressure_hpa_x100 = tank_rd ? (int32_t)lroundf(p_tank * 100.0f) : -1;
        bool level_ok = false;     /* true only when a real water level is known */

        if (baro_rd && tank_rd) {
            /* full hydrostatic measurement: hPa -> Pa -> depth */
            float corrected_delta_hpa = (p_tank - p_baro) - ((float)cfg.air_offset_hpa_x100 / 100.0f);
            float head_pa = corrected_delta_hpa * 100.0f;
            float depth_cm = head_pa / ((float)cfg.density * GRAVITY_MS2) * 100.0f + SENSOR_OFFSET_CM;
            if (depth_cm < 0) depth_cm = 0;
            g_depth_cm = (int16_t)lroundf(depth_cm);
            int16_t span = cfg.tank_h_cm > 0 ? cfg.tank_h_cm : 1;
            int pct = (int)lroundf(depth_cm * 100.0f / span);
            g_level_pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
#if BENCH_REQUIRE_EQUAL_PRESSURE
            if (fabsf(p_tank - p_baro) > BENCH_MAX_DELTA_HPA) {
                g_fault = 4;        /* 4 = bench sensors disagree */
                level_ok = false;
            } else
#endif
            if (!cfg.calibrated) {
                g_fault = 5;        /* 5 = setup/calibration incomplete */
                level_ok = false;
            } else
            if (g_depth_cm > cfg.tank_h_cm + MAX_DEPTH_OVER_TANK_CM) {
                g_fault = 3;        /* 3 = pressure delta is implausible */
                level_ok = false;
            } else {
                g_fault = 0;
                level_ok = true;
            }
            g_low_alert = (level_ok && g_depth_cm <= cfg.low_cm) ? 1 : 0;
            fault_count = 0;
#if LOG_SENSOR_READINGS
            ESP_LOGI(TAG, "sensor read: baro=%.2fhPa(%d/%d) tank=%.2fhPa(%d/%d) depth=%dcm level=%u%% fault=%u",
                     p_baro, baro_n, SENSOR_AVG_SAMPLES,
                     p_tank, tank_n, SENSOR_AVG_SAMPLES,
                     g_depth_cm, g_level_pct, g_fault);
#endif
        } else if (baro_rd || tank_rd) {
            /* DIAGNOSTIC: only one sensor present - cannot measure level.
             * Report that sensor's absolute pressure (hPa, ~1013 at sea level)
             * in 'depth' so it's visible over Zigbee = "this sensor is alive".
             * Relay stays in failsafe (no real level). */
            g_depth_cm = (int16_t)lroundf(baro_rd ? p_baro : p_tank);
            g_level_pct = 0;
            g_fault = 2;            /* 2 = single-sensor diagnostic mode */
            g_low_alert = 0;
            fault_count = 0;
#if LOG_SENSOR_READINGS
            ESP_LOGW(TAG, "sensor read: only %s OK, pressure=%.2fhPa",
                     baro_rd ? "baro" : "tank", baro_rd ? p_baro : p_tank);
#endif
        } else {
            if (++fault_count >= SENSOR_FAULT_LIMIT) g_fault = 1;
            g_low_alert = 0;
#if LOG_SENSOR_READINGS
            ESP_LOGW(TAG, "sensor read failed: baro=%s tank=%s fault_count=%d",
                     baro_ok ? "no-data" : "absent",
                     tank_ok ? "no-data" : "absent", fault_count);
#endif
        }

        /* ---- decide relay state ---- */
        int64_t now = esp_timer_get_time();
        bool want = g_relay_on;

        if (cfg.mode == MODE_ON) {
            want = true;
        } else if (cfg.mode == MODE_OFF) {
            want = false;
        } else if (level_ok) {                  /* AUTO with a real water level */
            if (g_depth_cm <= cfg.operating_cm) want = true;   /* operating -> fill */
            if (g_depth_cm >= cfg.full_cm) want = false;  /* full -> stop  */
        } else {                                /* AUTO, no full level available */
#if DIAG_RELAY_SENSOR_INDICATOR
            want = (baro_rd || tank_rd);        /* relay/LED ON = a sensor is reading */
#else
            want = false;                       /* failsafe */
#endif
        }

        /* anti-short-cycle guards (only when actually pump-controlling) */
        if (cfg.mode == MODE_AUTO && level_ok) {
            if (want && !g_relay_on) {
                if (now - last_off_us < (int64_t)PUMP_MIN_OFF_MS * 1000) want = false;
            }
            if (!want && g_relay_on && PUMP_MIN_ON_MS > 0) {
                if (now - last_on_us < (int64_t)PUMP_MIN_ON_MS * 1000) want = true;
            }
        }

        if (want != g_relay_on) {
            relay_set(want);
            if (want) last_on_us = now; else last_off_us = now;
            ESP_LOGI(TAG, "relay -> %s (depth=%dcm %d%% mode=%d fault=%d)",
                     want ? "ON" : "OFF", g_depth_cm, g_level_pct, cfg.mode, g_fault);
        }

        zb_push_telemetry();
        mqtt_publish_state();
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* ------------------------------ OTA client ------------------------------ */
/* Receives firmware over the Zigbee OTA cluster (served by Zigbee2MQTT) and
 * writes it to the spare app slot, then reboots into it. */
static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_ota_part = NULL;
static StreamBufferHandle_t s_ota_sb = NULL;
static uint32_t s_ota_bytes = 0;
static uint32_t s_ota_skip  = 0;            /* sub-element header bytes to drop  */
static volatile bool s_ota_finish = false;
static volatile bool s_ota_failed = false;
/* s_ota_active is declared up top so the telemetry gate can see it */
#define OTA_SUBELEMENT_HDR  6               /* tag id (2) + length (4)           */

/* Writer task: does all the slow flash erase/write off the radio callback, so
 * the Zigbee stack is never stalled (esp_ota_write inside the callback -> zb_assert). */
static void ota_writer_task(void *arg)
{
    s_ota_part = esp_ota_get_next_update_partition(NULL);
    esp_err_t err = s_ota_part ? esp_ota_begin(s_ota_part, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle)
                               : ESP_ERR_NOT_FOUND;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        s_ota_failed = true; s_ota_active = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "OTA writer -> %s", s_ota_part->label);

    uint8_t buf[512];
    while (1) {
        size_t n = xStreamBufferReceive(s_ota_sb, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (n > 0) {
            if (esp_ota_write(s_ota_handle, buf, n) != ESP_OK) { s_ota_failed = true; break; }
            s_ota_bytes += n;
        }
        if (s_ota_finish && xStreamBufferIsEmpty(s_ota_sb)) break;
    }

    if (!s_ota_failed && esp_ota_end(s_ota_handle) == ESP_OK
                      && esp_ota_set_boot_partition(s_ota_part) == ESP_OK) {
        ESP_LOGI(TAG, "OTA complete (%u bytes) - rebooting", (unsigned)s_ota_bytes);
        vTaskDelay(pdMS_TO_TICKS(1500));
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed/aborted (%u bytes)", (unsigned)s_ota_bytes);
        if (s_ota_handle) esp_ota_abort(s_ota_handle);
    }
    s_ota_handle = 0;
    s_ota_active = false;
    vTaskDelete(NULL);
}

static esp_err_t ota_value_cb(esp_zb_zcl_ota_upgrade_value_message_t msg)
{
    switch (msg.upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        if (s_ota_active) break;
        s_ota_bytes = 0;
        s_ota_skip = OTA_SUBELEMENT_HDR;
        s_ota_finish = false;
        s_ota_failed = false;
        if (!s_ota_sb) s_ota_sb = xStreamBufferCreate(16384, 1);
        if (!s_ota_sb) { s_ota_failed = true; break; }
        xStreamBufferReset(s_ota_sb);
        s_ota_active = true;
        xTaskCreate(ota_writer_task, "ota_wr", 8192, NULL, 4, NULL);
        ESP_LOGI(TAG, "OTA start (img ver 0x%08x)", (unsigned)msg.ota_header.file_version);
        break;
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        if (msg.payload_size && msg.payload && s_ota_active && !s_ota_failed) {
            const uint8_t *p = msg.payload;
            uint16_t n = msg.payload_size;
            if (s_ota_skip) {                       /* drop sub-element header */
                uint16_t s = s_ota_skip < n ? s_ota_skip : n;
                p += s; n -= s; s_ota_skip -= s;
            }
            if (n) xStreamBufferSend(s_ota_sb, p, n, pdMS_TO_TICKS(200));
        }
        break;
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
        break;                                  /* accept the image */
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
        s_ota_finish = true;                    /* writer task finalises + reboots */
        break;
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        ESP_LOGW(TAG, "OTA aborted by server");
        s_ota_failed = true;
        s_ota_finish = true;
        break;
    default:
        break;
    }
    return ESP_OK;
}

/* --------------------------- Zigbee write handler ----------------------- */
static esp_err_t zb_set_attr_cb(const esp_zb_zcl_set_attr_value_message_t *m)
{
    if (m->info.dst_endpoint != ZB_ENDPOINT) return ESP_OK;
    if (m->info.cluster != ZB_CUSTOM_CLUSTER_ID) return ESP_OK;

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    switch (m->attribute.id) {
        case ATTR_SET_LOW_CM:     g_cfg.low_cm    = *(int16_t *)m->attribute.data.value; break;
        case ATTR_SET_FULL_CM:    g_cfg.full_cm   = *(int16_t *)m->attribute.data.value; break;
        case ATTR_TANK_HEIGHT_CM: g_cfg.tank_h_cm = *(int16_t *)m->attribute.data.value; break;
        case ATTR_DENSITY:        g_cfg.density   = *(uint16_t *)m->attribute.data.value; break;
        case ATTR_MODE:           g_cfg.mode      = *(uint8_t *)m->attribute.data.value; break;
        case ATTR_OPERATING_CM:   g_cfg.operating_cm = *(int16_t *)m->attribute.data.value; break;
        default: break;
    }
    cfg_t snap = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    cfg_save();
    ESP_LOGI(TAG, "cfg update: lowAlert=%d operating=%d full=%d tankH=%d rho=%d mode=%d",
             snap.low_cm, snap.operating_cm, snap.full_cm, snap.tank_h_cm, snap.density, snap.mode);
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t cb_id, const void *message)
{
    if (cb_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return zb_set_attr_cb((const esp_zb_zcl_set_attr_value_message_t *)message);
    } else if (cb_id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) {
        return ota_value_cb(*(esp_zb_zcl_ota_upgrade_value_message_t *)message);
    } else if (cb_id == ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID) {
        const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *m =
            (const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message;
        ESP_LOGI(TAG, "OTA query response: status=%d version=0x%08x size=%u",
                 m->query_status, (unsigned)m->file_version, (unsigned)m->image_size);
    }
    return ESP_OK;
}

/* --------------------------- Zigbee device model ------------------------ */
static void add_custom_cluster(esp_zb_cluster_list_t *list)
{
    esp_zb_attribute_list_t *c = esp_zb_zcl_attr_list_create(ZB_CUSTOM_CLUSTER_ID);

    int16_t  s16 = 0;
    uint8_t  u8  = 0;
    uint16_t u16 = g_cfg.density;

    /* telemetry: read-only + reportable */
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_DEPTH_CM, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LEVEL_PCT, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &u8);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_FAULT, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &u8);
    s16 = -1;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_BARO_PRESSURE_HPA, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_TANK_PRESSURE_HPA, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LOW_ALERT, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &u8);

    /* config: read/write */
    s16 = g_cfg.low_cm;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SET_LOW_CM, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s16);
    s16 = g_cfg.full_cm;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_SET_FULL_CM, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s16);
    s16 = g_cfg.tank_h_cm;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_TANK_HEIGHT_CM, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_DENSITY, ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &u16);
    u8 = g_cfg.mode;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_MODE, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &u8);
    s16 = g_cfg.operating_cm;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_OPERATING_CM, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &s16);

    esp_zb_cluster_list_add_custom_cluster(list, c, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

static void esp_zb_task(void *arg)
{
    esp_zb_cfg_t nwk = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = { .max_children = 10 },
    };
    esp_zb_init(&nwk);

    /* ---- build endpoint ---- */
    esp_zb_basic_cluster_cfg_t   basic_cfg   = { .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
                                                 .power_source = 0x01 /* mains */ };
    esp_zb_identify_cluster_cfg_t ident_cfg  = { .identify_time = 0 };
    esp_zb_on_off_cluster_cfg_t  onoff_cfg   = { .on_off = 0 };

    esp_zb_cluster_list_t *clusters = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
    /* ZCL char strings are length-prefixed; these let Z2M match the converter */
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "\x03""DIY");
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "\x07""ESP32C5");
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, OTA_VERSION_ZCL_STRING);
    esp_zb_cluster_list_add_basic_cluster(clusters, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(clusters,
        esp_zb_identify_cluster_create(&ident_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(clusters,
        esp_zb_on_off_cluster_create(&onoff_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_pressure_meas_cluster_cfg_t pressure_cfg = {
        .measured_value = -1,
        .min_value = 0,
        .max_value = 1200,
    };
    esp_zb_attribute_list_t *pressure = esp_zb_pressure_meas_cluster_create(&pressure_cfg);
    int16_t scaled_value = -1;
    int16_t min_scaled_value = 0;
    int16_t max_scaled_value = 1200;
    uint16_t scaled_tolerance = 1;
    int8_t scale = 0;
    esp_zb_pressure_meas_cluster_add_attr(pressure, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID, &scaled_value);
    esp_zb_pressure_meas_cluster_add_attr(pressure, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_SCALED_VALUE_ID, &min_scaled_value);
    esp_zb_pressure_meas_cluster_add_attr(pressure, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_SCALED_VALUE_ID, &max_scaled_value);
    esp_zb_pressure_meas_cluster_add_attr(pressure, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_TOLERANCE_ID, &scaled_tolerance);
    esp_zb_pressure_meas_cluster_add_attr(pressure, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALE_ID, &scale);
    esp_zb_cluster_list_add_pressure_meas_cluster(clusters, pressure, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    add_custom_cluster(clusters);

    /* OTA upgrade client cluster - lets Zigbee2MQTT push firmware over the air */
    esp_zb_ota_cluster_cfg_t ota_cfg = {
        .ota_upgrade_file_version = OTA_FW_VERSION,
        .ota_upgrade_downloaded_file_ver = OTA_FW_VERSION,
        .ota_upgrade_manufacturer = OTA_MANUF_CODE,
        .ota_upgrade_image_type = OTA_IMAGE_TYPE,
        .ota_min_block_reque = 0,
        .ota_upgrade_file_offset = ESP_ZB_ZCL_OTA_UPGRADE_FILE_OFFSET_DEF_VALUE,
        .ota_upgrade_server_id = ESP_ZB_ZCL_OTA_UPGRADE_SERVER_DEF_VALUE,
        .ota_image_upgrade_status = ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_DEF_VALUE,
    };
    esp_zb_attribute_list_t *ota = esp_zb_ota_cluster_create(&ota_cfg);
    esp_zb_zcl_ota_upgrade_client_variable_t ota_var = {
        .timer_query = OTA_QUERY_INTERVAL_MIN,
        .hw_version = OTA_HW_VERSION,
        .max_data_size = OTA_MAX_DATA_SIZE,
    };
    esp_zb_ota_cluster_add_attr(ota, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &ota_var);
    esp_zb_cluster_list_add_ota_cluster(clusters, ota, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ZB_PRIMARY_CHANNEL_MASK);
    esp_zb_set_secondary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

static void zb_commissioning_alarm_cb(uint8_t mode)
{
    esp_zb_bdb_start_top_level_commissioning(mode);
}

/* signal handler: drives commissioning / network steering */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *sig)
{
    uint32_t *p = sig->p_app_signal;
    esp_err_t err = sig->esp_err_status;
    esp_zb_app_signal_type_t type = *p;

    switch (type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialised");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "joining network...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "rejoined existing network");
                g_zb_joined = true;
                g_join_us = esp_timer_get_time();
            }
        } else {
            ESP_LOGW(TAG, "init failed (%s), retrying", esp_err_to_name(err));
            esp_zb_scheduler_alarm(zb_commissioning_alarm_cb, ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err == ESP_OK) {
            g_zb_joined = true;
            g_join_us = esp_timer_get_time();
            esp_zb_ieee_addr_t ext;
            esp_zb_get_long_address(ext);
            ESP_LOGI(TAG, "joined, PAN 0x%04hx, ch %d", esp_zb_get_pan_id(),
                     esp_zb_get_current_channel());
        } else {
            ESP_LOGW(TAG, "no network found, retrying steering");
            esp_zb_scheduler_alarm(zb_commissioning_alarm_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGD(TAG, "ZDO signal 0x%x status %s", type, esp_err_to_name(err));
        break;
    }
}

/* ----------------------------- app_main --------------------------------- */
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    g_cfg_mtx = xSemaphoreCreateMutex();
    cfg_load();
    relay_init();                       /* relay OFF before anything else */

    bool setup_mode = false;
    bool wifi_mode = false;
#if SETUP_PORTAL_ENABLE
    setup_mode = (g_cfg.conn_mode == CONN_UNSET) || (g_cfg.conn_mode == CONN_AP) || setup_boot_button_held();
    if (setup_mode) {
        setup_portal_start();
    }
#endif

    bool start_zigbee = !setup_mode && (g_cfg.conn_mode == CONN_ZIGBEE);
    if (!setup_mode && g_cfg.conn_mode == CONN_WIFI) {
        wifi_mode = wifi_station_start();
        if (!wifi_mode) {
            setup_mode = true;
#if SETUP_PORTAL_ENABLE
            setup_portal_start();
#endif
        }
    }

    if (start_zigbee) {
        esp_zb_platform_config_t platform = {
            .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
            .host_config  = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
        };
        ESP_ERROR_CHECK(esp_zb_platform_config(&platform));
    }

    xTaskCreate(control_task, "control", 4096, NULL, 5, NULL);
    if (start_zigbee) {
        xTaskCreate(button_task, "button", 3072, NULL, 4, NULL);
        xTaskCreate(esp_zb_task, "zigbee", 8192, NULL, 6, NULL);
    }
}
