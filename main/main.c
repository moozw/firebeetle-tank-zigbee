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
#include <ctype.h>
#include <stdint.h>
#include <limits.h>
#include <stddef.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "mbedtls/base64.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"

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
    uint8_t  lockout_enabled;
    uint16_t lockout_start_min;
    uint16_t lockout_end_min;
    char     web_pass[33];      /* config-page admin password (WiFi mode) */
    uint8_t  web_pass_changed;  /* 0 = still the shipped default          */
} cfg_t;

static cfg_t g_cfg = {
    .magic     = 0x544b,     /* "TK" */
    .version   = 6,
    .calibrated = 0,
    .low_cm    = DEFAULT_LEVEL_LOW_CM,
    .operating_cm = DEFAULT_OPERATING_CM,
    .full_cm   = DEFAULT_LEVEL_FULL_CM,
    .tank_h_cm = DEFAULT_TANK_HEIGHT_CM,
    .density   = DEFAULT_WATER_DENSITY,
    .mode      = MODE_AUTO,
    .conn_mode = CONN_UNSET,
    .lockout_enabled = 0,
    .lockout_start_min = DEFAULT_LOCKOUT_START_MIN,
    .lockout_end_min = DEFAULT_LOCKOUT_END_MIN,
    .air_offset_hpa_x100 = 0,
    .wifi_ssid = "",
    .wifi_pass = "",
    .mqtt_host = DEFAULT_MQTT_HOST,
    .mqtt_user = "",
    .mqtt_pass = "",
    .mqtt_topic = DEFAULT_MQTT_TOPIC,
    .web_pass = SETUP_WEB_DEFAULT_PASS,
    .web_pass_changed = 0,
};
static SemaphoreHandle_t g_cfg_mtx;
static SemaphoreHandle_t g_sensor_mtx;
static i2c_master_bus_handle_t g_sensor_bus;
static lps2x_t g_baro_sensor;
static lps2x_t g_tank_sensor;
static bool g_baro_sensor_ok = false;
static bool g_tank_sensor_ok = false;
static void cfg_save(void);
static bool sensor_pressure_valid(float hpa);
static void time_sync_start(void);
static bool time_sync_request_resync(void);

static const cfg_t DEFAULT_CFG = {
    .magic     = 0x544b,
    .version   = 6,
    .calibrated = 0,
    .low_cm    = DEFAULT_LEVEL_LOW_CM,
    .operating_cm = DEFAULT_OPERATING_CM,
    .full_cm   = DEFAULT_LEVEL_FULL_CM,
    .tank_h_cm = DEFAULT_TANK_HEIGHT_CM,
    .density   = DEFAULT_WATER_DENSITY,
    .mode      = MODE_AUTO,
    .conn_mode = CONN_UNSET,
    .lockout_enabled = 0,
    .lockout_start_min = DEFAULT_LOCKOUT_START_MIN,
    .lockout_end_min = DEFAULT_LOCKOUT_END_MIN,
    .air_offset_hpa_x100 = 0,
    .wifi_ssid = "",
    .wifi_pass = "",
    .mqtt_host = DEFAULT_MQTT_HOST,
    .mqtt_user = "",
    .mqtt_pass = "",
    .mqtt_topic = DEFAULT_MQTT_TOPIC,
    .web_pass = SETUP_WEB_DEFAULT_PASS,
    .web_pass_changed = 0,
};

static const char *cfg_validate(const cfg_t *cfg, bool require_wifi_ssid)
{
    if (cfg->magic != DEFAULT_CFG.magic || cfg->version != DEFAULT_CFG.version) return "version";
    if (cfg->calibrated > 1) return "calibrated";
    if (cfg->low_cm < 0) return "low_cm";
    if (cfg->operating_cm < 0) return "operating_cm";
    if (cfg->operating_cm < cfg->low_cm) return "operating_below_low";
    if (cfg->full_cm <= cfg->operating_cm) return "full_below_operating";
    if (cfg->tank_h_cm < 10 || cfg->tank_h_cm > 1000) return "tank_height";
    if (cfg->full_cm > cfg->tank_h_cm) return "full_above_tank";
    if (cfg->density < 900 || cfg->density > 1100) return "density";
    if (cfg->mode > MODE_OFF) return "mode";
    if (cfg->conn_mode != CONN_UNSET && cfg->conn_mode > CONN_WIFI) return "conn_mode";
    if (require_wifi_ssid && cfg->conn_mode == CONN_WIFI && !cfg->wifi_ssid[0]) return "wifi_ssid";
    if (cfg->lockout_enabled > 1) return "lockout_enabled";
    if (cfg->lockout_start_min >= 1440 || cfg->lockout_end_min >= 1440) return "lockout_time";
    if (cfg->wifi_ssid[sizeof(cfg->wifi_ssid) - 1] != '\0') return "wifi_ssid_terminated";
    if (cfg->wifi_pass[sizeof(cfg->wifi_pass) - 1] != '\0') return "wifi_pass_terminated";
    if (cfg->mqtt_host[sizeof(cfg->mqtt_host) - 1] != '\0') return "mqtt_host_terminated";
    if (cfg->mqtt_user[sizeof(cfg->mqtt_user) - 1] != '\0') return "mqtt_user_terminated";
    if (cfg->mqtt_pass[sizeof(cfg->mqtt_pass) - 1] != '\0') return "mqtt_pass_terminated";
    if (cfg->mqtt_topic[sizeof(cfg->mqtt_topic) - 1] != '\0') return "mqtt_topic_terminated";
    if (cfg->web_pass_changed > 1) return "web_pass_changed";
    if (cfg->web_pass[sizeof(cfg->web_pass) - 1] != '\0') return "web_pass_terminated";
    return NULL;
}

static bool cfg_clamp_level_safety(cfg_t *cfg)
{
    bool changed = false;
    if (cfg->tank_h_cm >= 10 && cfg->full_cm > cfg->tank_h_cm) {
        cfg->full_cm = cfg->tank_h_cm;
        changed = true;
    }
    if (cfg->operating_cm >= cfg->full_cm) {
        cfg->operating_cm = cfg->full_cm > 0 ? cfg->full_cm - 1 : 0;
        changed = true;
    }
    if (cfg->low_cm > cfg->operating_cm) {
        cfg->low_cm = cfg->operating_cm;
        changed = true;
    }
    return changed;
}

/* live telemetry (updated by control task) */
static int16_t g_depth_cm = 0;
static int16_t g_baro_pressure_hpa = -1;
static int16_t g_tank_pressure_hpa = -1;
static int32_t g_baro_pressure_hpa_x100 = -1;
static int32_t g_tank_pressure_hpa_x100 = -1;
static int16_t g_air_temp_c_x100 = INT16_MIN;
static int16_t g_water_temp_c_x100 = INT16_MIN;
static uint8_t g_level_pct = 0;
static uint8_t g_fault     = 0;
static uint8_t g_low_alert = 0;
static uint8_t g_lockout_active = 0;
static uint8_t g_time_valid = 0;
static time_t g_last_time_sync_epoch = 0;
static int64_t g_last_time_sync_us = 0;
static uint32_t g_time_sync_count = 0;
static bool    g_relay_on  = false;
static volatile bool g_zb_joined = false;   /* true once on a Zigbee network */
static volatile int64_t g_join_us = 0;      /* timestamp (us) when we joined   */
static volatile bool s_ota_active = false;  /* true while an OTA is downloading */
static volatile bool s_zb_started = false;  /* true once the Zigbee stack is up  */
static volatile bool s_wifi_connected = false;
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;
static char s_mqtt_uri[96];
static httpd_handle_t s_httpd = NULL;
static int s_wifi_retry = 0;
static volatile bool s_wifi_started = false;
static volatile bool s_setup_portal_active = false;
static volatile bool s_web_auth = false;    /* require Basic auth (set in WiFi/STA mode) */

/* ----------------------------- NVS persistence -------------------------- */
static void cfg_load(void)
{
    nvs_handle_t h;
    if (nvs_open("tank", NVS_READONLY, &h) != ESP_OK) return;
    size_t sz = 0;
    esp_err_t err = nvs_get_blob(h, "cfg", NULL, &sz);
    bool migrated = false;
    if (err == ESP_OK && sz <= sizeof(g_cfg)) {
        g_cfg = DEFAULT_CFG;
        err = nvs_get_blob(h, "cfg", &g_cfg, &sz);
        if (err == ESP_OK && g_cfg.version == 4) {
            g_cfg.lockout_enabled = 0;
            g_cfg.lockout_start_min = DEFAULT_LOCKOUT_START_MIN;
            g_cfg.lockout_end_min = DEFAULT_LOCKOUT_END_MIN;
            g_cfg.version = 5;
            migrated = true;
        }
        if (err == ESP_OK && g_cfg.version == 5) {
            /* web_pass/web_pass_changed already defaulted from DEFAULT_CFG since
             * they sit past the end of the older (v5) stored blob */
            g_cfg.version = 6;
            migrated = true;
        }
        if (err == ESP_OK && cfg_clamp_level_safety(&g_cfg)) {
            migrated = true;
        }
    }
    nvs_close(h);

    const char *bad = (err != ESP_OK || sz > sizeof(g_cfg)) ? "nvs_blob" : cfg_validate(&g_cfg, false);
    if (bad) {
        ESP_LOGW(TAG, "stored config invalid (%s), restoring defaults", bad);
        g_cfg = DEFAULT_CFG;
        cfg_save();
    } else if (migrated) {
        ESP_LOGI(TAG, "stored config migrated to version %u", DEFAULT_CFG.version);
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

/* In-place URL-decode of an x-www-form-urlencoded value (%XX escapes and '+'). */
static void url_decode(char *s)
{
    char *o = s;
    for (char *p = s; *p; p++) {
        if (*p == '%' && isxdigit((unsigned char)p[1]) && isxdigit((unsigned char)p[2])) {
            char hex[3] = { p[1], p[2], '\0' };
            *o++ = (char)strtol(hex, NULL, 16);
            p += 2;
        } else if (*p == '+') {
            *o++ = ' ';
        } else {
            *o++ = *p;
        }
    }
    *o = '\0';
}

static void copy_form_str(const char *form, const char *key, char *dst, size_t dst_len)
{
    char val[96];
    if (dst_len == 0) return;
    if (httpd_query_key_value(form, key, val, sizeof(val)) == ESP_OK) {
        url_decode(val);                 /* httpd_query_key_value does NOT decode */
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

static void json_escape_str(const char *src, char *dst, size_t dst_len)
{
    if (dst_len == 0) return;
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p && o + 1 < dst_len; p++) {
        if ((*p == '"' || *p == '\\') && o + 2 < dst_len) {
            dst[o++] = '\\';
            dst[o++] = (char)*p;
        } else if (*p >= 0x20) {
            dst[o++] = (char)*p;
        }
    }
    dst[o] = '\0';
}

static esp_err_t read_request_body(httpd_req_t *req, char *buf, size_t buf_len)
{
    if (buf_len == 0) return ESP_ERR_INVALID_SIZE;
    if (req->content_len >= buf_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t off = 0;
    while (off < req->content_len) {
        int got = httpd_req_recv(req, buf + off, req->content_len - off);
        if (got <= 0) return ESP_FAIL;
        off += (size_t)got;
    }
    buf[off] = '\0';
    return ESP_OK;
}

static bool clock_is_valid(time_t now)
{
    return now >= 1704067200; /* 2024-01-01 UTC: before this, SNTP/RTC is not sane */
}

static void format_tm_iso(const struct tm *tmv, const char *suffix, char *out, size_t out_len)
{
    if (!tmv || out_len == 0) return;
    int n = snprintf(out, out_len, "%04d-%02d-%02dT%02d:%02d:%02d%s",
                     tmv->tm_year + 1900, tmv->tm_mon + 1, tmv->tm_mday,
                     tmv->tm_hour, tmv->tm_min, tmv->tm_sec, suffix ? suffix : "");
    if (n < 0 || n >= (int)out_len) out[out_len - 1] = '\0';
}

static void format_minute_hhmm(uint16_t minute, char *out, size_t out_len)
{
    if (out_len == 0) return;
    if (minute >= 1440) {
        strlcpy(out, "??:??", out_len);
        return;
    }
    snprintf(out, out_len, "%02u:%02u", (unsigned)(minute / 60), (unsigned)(minute % 60));
}

static int32_t local_utc_offset_min(const struct tm *utc, const struct tm *local)
{
    if (!utc || !local) return 0;
    struct tm utc_as_local = *utc;
    struct tm local_copy = *local;
    time_t utc_epoch_local = mktime(&utc_as_local);
    time_t local_epoch = mktime(&local_copy);
    if (utc_epoch_local == (time_t)-1 || local_epoch == (time_t)-1) return 0;
    return (int32_t)(difftime(local_epoch, utc_epoch_local) / 60.0);
}

static const char *sntp_status_name(sntp_sync_status_t status)
{
    switch (status) {
        case SNTP_SYNC_STATUS_RESET: return "reset";
        case SNTP_SYNC_STATUS_COMPLETED: return "completed";
        case SNTP_SYNC_STATUS_IN_PROGRESS: return "in_progress";
        default: return "unknown";
    }
}

static void time_sync_notification_cb(struct timeval *tv)
{
    g_last_time_sync_epoch = tv ? tv->tv_sec : time(NULL);
    g_last_time_sync_us = esp_timer_get_time();
    g_time_sync_count++;
    ESP_LOGI(TAG, "SNTP sync #%lu epoch=%lld", (unsigned long)g_time_sync_count,
             (long long)g_last_time_sync_epoch);
}

static bool local_time_minutes(uint16_t *minute_of_day)
{
    time_t now = time(NULL);
    if (!clock_is_valid(now)) {
        g_time_valid = 0;
        return false;
    }
    struct tm local;
    localtime_r(&now, &local);
    *minute_of_day = (uint16_t)(local.tm_hour * 60 + local.tm_min);
    g_time_valid = 1;
    return true;
}

static bool minute_in_window(uint16_t minute, uint16_t start, uint16_t end)
{
    if (start == end) return false;
    if (start < end) return minute >= start && minute < end;
    return minute >= start || minute < end;
}

static bool lockout_is_active(const cfg_t *cfg)
{
    if (!cfg->lockout_enabled) {
        g_lockout_active = 0;
        if (cfg->conn_mode == CONN_WIFI) {
            uint16_t minute = 0;
            (void)local_time_minutes(&minute);
        } else {
            g_time_valid = 0;
        }
        return false;
    }

    if (cfg->conn_mode == CONN_WIFI) {
        uint16_t minute = 0;
        if (!local_time_minutes(&minute)) {
            g_lockout_active = 1; /* safe fallback: no clock, no automatic pump run */
            return true;
        }
        g_lockout_active = minute_in_window(minute, cfg->lockout_start_min, cfg->lockout_end_min) ? 1 : 0;
        return g_lockout_active != 0;
    }

    /* Zigbee/Home Assistant mode: the coordinator sets this directly. */
    g_time_valid = 0;
    g_lockout_active = 1;
    return true;
}

static bool sensor_pressure_valid(float hpa)
{
    return isfinite(hpa) && hpa >= SENSOR_MIN_VALID_HPA;
}

static float median_float(float *values, int count)
{
    for (int i = 1; i < count; i++) {
        float v = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > v) {
            values[j + 1] = values[j];
            j--;
        }
        values[j + 1] = v;
    }
    if ((count & 1) != 0) {
        return values[count / 2];
    }
    return (values[count / 2 - 1] + values[count / 2]) / 2.0f;
}

static void mqtt_publish_state(void)
{
    if (!s_mqtt || !s_mqtt_connected) return;

    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char topic[64];
    char payload[560];
    snprintf(topic, sizeof(topic), "%s/state", cfg.mqtt_topic[0] ? cfg.mqtt_topic : DEFAULT_MQTT_TOPIC);
    int n = snprintf(payload, sizeof(payload),
        "{\"depth_cm\":%d,\"level_pct\":%u,\"fault\":%u,\"low_alert\":%u,"
        "\"relay\":%s,\"mode\":%u,\"low_cm\":%d,\"operating_cm\":%d,\"full_cm\":%d,"
        "\"lockout_enabled\":%u,\"lockout_start_min\":%u,\"lockout_end_min\":%u,"
        "\"lockout_active\":%u,\"time_valid\":%u,"
        "\"baro_hpa\":%.2f,\"tank_hpa\":%.2f,\"air_temp_c\":%.2f,\"water_temp_c\":%.2f}",
        g_depth_cm, g_level_pct, g_fault, g_low_alert, g_relay_on ? "true" : "false",
        cfg.mode, cfg.low_cm, cfg.operating_cm, cfg.full_cm,
        cfg.lockout_enabled, cfg.lockout_start_min, cfg.lockout_end_min, g_lockout_active, g_time_valid,
        g_baro_pressure_hpa_x100 >= 0 ? (double)g_baro_pressure_hpa_x100 / 100.0 : -1.0,
        g_tank_pressure_hpa_x100 >= 0 ? (double)g_tank_pressure_hpa_x100 / 100.0 : -1.0,
        g_air_temp_c_x100 != INT16_MIN ? (double)g_air_temp_c_x100 / 100.0 : -99.99,
        g_water_temp_c_x100 != INT16_MIN ? (double)g_water_temp_c_x100 / 100.0 : -99.99);
    if (n < 0) return;
    if (n >= (int)sizeof(payload)) n = strlen(payload);
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
    if (json_has(json, "pump_lockout", val, sizeof(val)) ||
        json_has(json, "lockout_enabled", val, sizeof(val))) {
        cfg.lockout_enabled = (strcmp(val, "true") == 0 || strcmp(val, "on") == 0 || atoi(val) != 0) ? 1 : 0;
    }
    if (json_has(json, "lockout_start_min", val, sizeof(val))) cfg.lockout_start_min = atoi(val);
    if (json_has(json, "lockout_end_min", val, sizeof(val))) cfg.lockout_end_min = atoi(val);

    const char *bad = cfg_validate(&cfg, false);
    if (bad) {
        ESP_LOGW(TAG, "MQTT command rejected (%s): %s", bad, json);
        return;
    }

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg.low_cm = cfg.low_cm;
    g_cfg.operating_cm = cfg.operating_cm;
    g_cfg.full_cm = cfg.full_cm;
    g_cfg.mode = cfg.mode;
    g_cfg.lockout_enabled = cfg.lockout_enabled;
    g_cfg.lockout_start_min = cfg.lockout_start_min;
    g_cfg.lockout_end_min = cfg.lockout_end_min;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    ESP_LOGI(TAG, "MQTT cfg: lowAlert=%d operating=%d full=%d mode=%u lockout=%u",
             cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.mode, cfg.lockout_enabled);
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
        .crt_bundle_attach = esp_crt_bundle_attach,  /* verify TLS (e.g. GitHub raw) */
        /* GitHub release assets 302-redirect to a CDN whose response headers and
         * long signed redirect URL overflow the 512-byte default -> "Out of
         * buffer". Enlarge both header buffers so the redirect can be followed. */
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
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

    /* Update the local attribute values only (check_change=false). The hub's
     * configureReporting (Z2M "Reconfigure") drives the periodic reports. We must
     * NOT issue manual esp_zb_zcl_report_attr_cmd_req (asserts in
     * zcl_general_commands.c:612) nor use check_change=true (returns an error
     * every cycle and stalls telemetry). */
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_DEPTH_CM, &g_depth_cm, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LEVEL_PCT, &g_level_pct, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_FAULT, &g_fault, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LOW_ALERT, &g_low_alert, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_BARO_PRESSURE_HPA, &g_baro_pressure_hpa, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TANK_PRESSURE_HPA, &g_tank_pressure_hpa, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_AIR_TEMP_CX100, &g_air_temp_c_x100, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_WATER_TEMP_CX100, &g_water_temp_c_x100, false);
    uint8_t relay = g_relay_on ? 1 : 0;
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_RELAY_STATE, &relay, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_LOCKOUT_ACTIVE, &g_lockout_active, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_TIME_VALID, &g_time_valid, false);
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg_t cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);
    uint8_t lockout = cfg.lockout_enabled ? 1 : 0;
    uint8_t mode = cfg.mode;
    /* The standard on/off switch is the AUTO lockout input, not a direct relay
     * command. FORCE ON and FORCE OFF remain controller modes. */
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &lockout, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &g_baro_pressure_hpa, false);
    esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_SCALED_VALUE_ID, &g_tank_pressure_hpa, false);

    /* standard reportable sensor endpoints (these DO auto-report) */
    float depth_f = (float)g_depth_cm;
    float level_f = (float)g_level_pct;
    float fault_f = (float)g_fault;
    float baro_f = (float)g_baro_pressure_hpa;
    float tank_f = (float)g_tank_pressure_hpa;
    float low_alert_f = (float)g_low_alert;
    float relay_f = relay ? 1.0f : 0.0f;
    float mode_f = (float)mode;
    float low_set_f = (float)cfg.low_cm;
    float operating_set_f = (float)cfg.operating_cm;
    float full_set_f = (float)cfg.full_cm;
    float tank_height_set_f = (float)cfg.tank_h_cm;
    float density_set_f = (float)cfg.density;
    esp_zb_zcl_set_attribute_val(ZB_EP_DEPTH, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &depth_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_LEVEL, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &level_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_WATER_TEMP, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &g_water_temp_c_x100, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_AIR_TEMP, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &g_air_temp_c_x100, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_FAULT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &fault_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_BARO_PRESSURE, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &baro_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_TANK_PRESSURE, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &tank_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_LOW_ALERT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &low_alert_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_RELAY, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &relay_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_MODE, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &mode_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_LOW, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &low_set_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_OPERATING, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &operating_set_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_FULL, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &full_set_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_TANK_HEIGHT, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &tank_height_set_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_DENSITY, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &density_set_f, false);
    esp_zb_zcl_set_attribute_val(ZB_EP_SET_MODE, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &mode_f, false);

    esp_zb_lock_release();
}

/* push the current mode to Zigbee so the hub reflects a local button change */
static void zb_push_mode(uint8_t mode)
{
    if (!zb_ready()) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_set_attribute_val(ZB_ENDPOINT, ZB_CUSTOM_CLUSTER_ID,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_MODE, &mode, false);
    float mode_f = (float)mode;
    esp_err_t standard_err = esp_zb_zcl_set_attribute_val(ZB_EP_SET_MODE,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID, &mode_f, false);
    esp_err_t report_err = esp_zb_zcl_set_attribute_val(ZB_EP_MODE,
        ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &mode_f, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee mode attr update failed: %s", esp_err_to_name(err));
    }
    if (standard_err != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee standard mode update failed: %s", esp_err_to_name(standard_err));
    }
    if (report_err != ESP_OK) {
        ESP_LOGW(TAG, "Zigbee mode report endpoint update failed: %s", esp_err_to_name(report_err));
    }
    esp_zb_lock_release();
}

/* ----------------------------- setup portal ----------------------------- */
#if SETUP_PORTAL_ENABLE

/* HTTP Basic auth, enforced only when s_web_auth is set (local-WiFi/STA mode).
 * Username is fixed (SETUP_WEB_USER); password is cfg.web_pass. Returns true if
 * the request is allowed to proceed. */
static bool web_auth_ok(httpd_req_t *req)
{
    if (!s_web_auth) return true;            /* setup AP: open */

    char hdr[160];
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK) return false;
    if (strncmp(hdr, "Basic ", 6) != 0) return false;

    unsigned char dec[128];
    size_t olen = 0;
    const char *b64 = hdr + 6;
    if (mbedtls_base64_decode(dec, sizeof(dec) - 1, &olen, (const unsigned char *)b64, strlen(b64)) != 0)
        return false;
    dec[olen] = '\0';

    char expect[100];
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    int n = snprintf(expect, sizeof(expect), "%s:%s", SETUP_WEB_USER, g_cfg.web_pass);
    xSemaphoreGive(g_cfg_mtx);
    if (n < 0 || n >= (int)sizeof(expect)) return false;
    return strcmp((char *)dec, expect) == 0;
}

static esp_err_t web_auth_challenge(httpd_req_t *req)
{
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Tank Controller\"");
    return httpd_resp_send(req, "Authentication required", HTTPD_RESP_USE_STRLEN);
}

#define WEB_REQUIRE_AUTH(req) do { if (!web_auth_ok(req)) return web_auth_challenge(req); } while (0)

static esp_err_t setup_status_get(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);
    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    char wifi_ssid[sizeof(cfg.wifi_ssid) * 2];
    char mqtt_host[sizeof(cfg.mqtt_host) * 2];
    char mqtt_user[sizeof(cfg.mqtt_user) * 2];
    char mqtt_topic[sizeof(cfg.mqtt_topic) * 2];
    json_escape_str(cfg.wifi_ssid, wifi_ssid, sizeof(wifi_ssid));
    json_escape_str(cfg.mqtt_host, mqtt_host, sizeof(mqtt_host));
    json_escape_str(cfg.mqtt_user, mqtt_user, sizeof(mqtt_user));
    json_escape_str(cfg.mqtt_topic, mqtt_topic, sizeof(mqtt_topic));

    char json[1220];
    int n = snprintf(json, sizeof(json),
        "{\"baro_hpa\":%.2f,\"tank_hpa\":%.2f,\"air_temp_c\":%.2f,\"water_temp_c\":%.2f,"
        "\"depth_cm\":%d,\"level_pct\":%u,"
        "\"fault\":%u,\"low_alert\":%u,\"relay\":%s,\"calibrated\":%u,\"air_offset_hpa\":%.2f,"
        "\"low_cm\":%d,\"operating_cm\":%d,\"full_cm\":%d,\"tank_height_cm\":%d,\"density\":%u,"
        "\"mode\":%u,\"conn_mode\":%u,\"lockout_enabled\":%u,\"lockout_start_min\":%u,"
        "\"lockout_end_min\":%u,\"lockout_active\":%u,\"time_valid\":%u,"
        "\"web_pass_changed\":%u,\"wifi_auth\":%s,"
        "\"wifi_connected\":%s,\"mqtt_connected\":%s,"
        "\"wifi_ssid\":\"%s\",\"mqtt_host\":\"%s\",\"mqtt_user\":\"%s\",\"mqtt_topic\":\"%s\"}",
        g_baro_pressure_hpa_x100 >= 0 ? (double)g_baro_pressure_hpa_x100 / 100.0 : -1.0,
        g_tank_pressure_hpa_x100 >= 0 ? (double)g_tank_pressure_hpa_x100 / 100.0 : -1.0,
        g_air_temp_c_x100 != INT16_MIN ? (double)g_air_temp_c_x100 / 100.0 : -99.99,
        g_water_temp_c_x100 != INT16_MIN ? (double)g_water_temp_c_x100 / 100.0 : -99.99,
        g_depth_cm, g_level_pct, g_fault, g_low_alert, g_relay_on ? "true" : "false",
        cfg.calibrated, (double)cfg.air_offset_hpa_x100 / 100.0,
        cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.tank_h_cm, cfg.density, cfg.mode, cfg.conn_mode,
        cfg.lockout_enabled, cfg.lockout_start_min, cfg.lockout_end_min, g_lockout_active, g_time_valid,
        cfg.web_pass_changed, s_web_auth ? "true" : "false",
        s_wifi_connected ? "true" : "false", s_mqtt_connected ? "true" : "false",
        wifi_ssid, mqtt_host, mqtt_user, mqtt_topic);
    if (n < 0) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Status format failed");
    if (n >= (int)sizeof(json)) n = strlen(json);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

static void json_float_or_null(char *out, size_t out_len, bool ok, float value)
{
    if (ok) {
        snprintf(out, out_len, "%.2f", (double)value);
    } else {
        strlcpy(out, "null", out_len);
    }
}

static esp_err_t setup_raw_sensor_get(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);

    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    bool ready = false;
    bool baro_present = false;
    bool tank_present = false;
    esp_err_t baro_err = ESP_ERR_INVALID_STATE;
    esp_err_t tank_err = ESP_ERR_INVALID_STATE;
    float p_baro = 0, t_baro = 0;
    float p_tank = 0, t_tank = 0;

    if (g_sensor_mtx && xSemaphoreTake(g_sensor_mtx, pdMS_TO_TICKS(5000)) == pdTRUE) {
        ready = (g_sensor_bus != NULL);
        if (ready) {
            baro_present = g_baro_sensor_ok;
            tank_present = g_tank_sensor_ok;
            baro_err = baro_present ? lps2x_read(&g_baro_sensor, &p_baro, &t_baro) : ESP_ERR_NOT_FOUND;
            tank_err = tank_present ? lps2x_read(&g_tank_sensor, &p_tank, &t_tank) : ESP_ERR_NOT_FOUND;
        }
        xSemaphoreGive(g_sensor_mtx);
    }

    bool baro_ok = (baro_err == ESP_OK);
    bool tank_ok = (tank_err == ESP_OK);
    bool baro_valid = baro_ok && sensor_pressure_valid(p_baro);
    bool tank_valid = tank_ok && sensor_pressure_valid(p_tank);
    bool level_ok = baro_valid && tank_valid;
    float delta_hpa = level_ok ? (p_tank - p_baro) - ((float)cfg.air_offset_hpa_x100 / 100.0f) : 0.0f;
    float depth_cm = level_ok
        ? (delta_hpa * 100.0f) / ((float)cfg.density * GRAVITY_MS2) * 100.0f + SENSOR_OFFSET_CM
        : 0.0f;
    if (depth_cm < 0) depth_cm = 0;
    int pct = level_ok && cfg.tank_h_cm > 0 ? (int)lroundf(depth_cm * 100.0f / cfg.tank_h_cm) : 0;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    char baro_hpa[24], baro_temp[24], tank_hpa[24], tank_temp[24], delta[24], depth[24];
    json_float_or_null(baro_hpa, sizeof(baro_hpa), baro_ok, p_baro);
    json_float_or_null(baro_temp, sizeof(baro_temp), baro_ok, t_baro);
    json_float_or_null(tank_hpa, sizeof(tank_hpa), tank_ok, p_tank);
    json_float_or_null(tank_temp, sizeof(tank_temp), tank_ok, t_tank);
    json_float_or_null(delta, sizeof(delta), level_ok, delta_hpa);
    json_float_or_null(depth, sizeof(depth), level_ok, depth_cm);

    char json[900];
    int n = snprintf(json, sizeof(json),
        "{\"raw\":true,\"ready\":%s,"
        "\"baro\":{\"addr\":%u,\"present\":%s,\"ok\":%s,\"err\":\"%s\",\"hpa\":%s,\"temp_c\":%s,\"valid\":%s},"
        "\"tank\":{\"addr\":%u,\"present\":%s,\"ok\":%s,\"err\":\"%s\",\"hpa\":%s,\"temp_c\":%s,\"valid\":%s},"
        "\"level_ok\":%s,\"delta_hpa\":%s,\"depth_cm\":%s,\"level_pct\":%d,"
        "\"controller_fault\":%u,\"controller_relay\":%s}",
        ready ? "true" : "false",
        LPS_BARO_ADDR, baro_present ? "true" : "false", baro_ok ? "true" : "false",
        esp_err_to_name(baro_err), baro_hpa, baro_temp, baro_valid ? "true" : "false",
        LPS_TANK_ADDR, tank_present ? "true" : "false", tank_ok ? "true" : "false",
        esp_err_to_name(tank_err), tank_hpa, tank_temp, tank_valid ? "true" : "false",
        level_ok ? "true" : "false", delta, depth, pct,
        g_fault, g_relay_on ? "true" : "false");
    if (n < 0) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Raw sensor format failed");
    if (n >= (int)sizeof(json)) n = strlen(json);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

static esp_err_t setup_clock_get(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);

    cfg_t cfg;
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    time_t now = time(NULL);
    bool valid = clock_is_valid(now);
    struct tm utc = {0};
    struct tm local = {0};
    char utc_s[32] = "";
    char local_s[32] = "";
    char last_sync_s[32] = "";
    uint16_t utc_minute = 0;
    uint16_t local_minute = 0;
    int32_t utc_offset_min = 0;

    if (valid) {
        gmtime_r(&now, &utc);
        localtime_r(&now, &local);
        utc_minute = (uint16_t)(utc.tm_hour * 60 + utc.tm_min);
        local_minute = (uint16_t)(local.tm_hour * 60 + local.tm_min);
        utc_offset_min = local_utc_offset_min(&utc, &local);
        format_tm_iso(&utc, "Z", utc_s, sizeof(utc_s));
        format_tm_iso(&local, "", local_s, sizeof(local_s));
    }
    if (clock_is_valid(g_last_time_sync_epoch)) {
        struct tm last_utc = {0};
        gmtime_r(&g_last_time_sync_epoch, &last_utc);
        format_tm_iso(&last_utc, "Z", last_sync_s, sizeof(last_sync_s));
    }

    bool lockout_calc_local = cfg.lockout_enabled && cfg.conn_mode == CONN_WIFI && valid &&
        minute_in_window(local_minute, cfg.lockout_start_min, cfg.lockout_end_min);
    bool lockout_calc_utc = cfg.lockout_enabled && cfg.conn_mode == CONN_WIFI && valid &&
        minute_in_window(utc_minute, cfg.lockout_start_min, cfg.lockout_end_min);
    char local_hhmm[8], utc_hhmm[8], start_hhmm[8], end_hhmm[8];
    format_minute_hhmm(local_minute, local_hhmm, sizeof(local_hhmm));
    format_minute_hhmm(utc_minute, utc_hhmm, sizeof(utc_hhmm));
    format_minute_hhmm(cfg.lockout_start_min, start_hhmm, sizeof(start_hhmm));
    format_minute_hhmm(cfg.lockout_end_min, end_hhmm, sizeof(end_hhmm));

    bool sntp_enabled = esp_sntp_enabled();
    sntp_sync_status_t sync_status = esp_sntp_get_sync_status();
    int64_t since_sync_s = g_last_time_sync_us > 0
        ? (esp_timer_get_time() - g_last_time_sync_us) / 1000000
        : -1;
    const char *server = esp_sntp_getservername(0);
    const char *tz = getenv("TZ");

    char json[1400];
    int n = snprintf(json, sizeof(json),
        "{\"epoch\":%lld,\"valid\":%s,"
        "\"timezone\":\"%s\",\"timezone_name\":\"%s\",\"utc_offset_min\":%ld,"
        "\"utc\":\"%s\",\"local\":\"%s\",\"utc_minute\":%u,\"local_minute\":%u,"
        "\"utc_hhmm\":\"%s\",\"local_hhmm\":\"%s\","
        "\"lockout_enabled\":%u,\"lockout_start_min\":%u,\"lockout_end_min\":%u,"
        "\"lockout_start_hhmm\":\"%s\",\"lockout_end_hhmm\":\"%s\","
        "\"lockout_active\":%u,\"lockout_calc_local\":%s,\"lockout_calc_utc\":%s,"
        "\"conn_mode\":%u,\"time_valid\":%u,"
        "\"sntp_enabled\":%s,\"sntp_status\":\"%s\",\"sntp_server\":\"%s\","
        "\"sntp_sync_interval_ms\":%lu,\"sntp_reachability\":%u,"
        "\"last_sync_epoch\":%lld,\"last_sync_utc\":\"%s\","
        "\"seconds_since_last_sync\":%lld,\"sync_count\":%lu}",
        (long long)now, valid ? "true" : "false",
        tz ? tz : "", LOCAL_TIMEZONE_NAME, (long)utc_offset_min,
        utc_s, local_s, utc_minute, local_minute, utc_hhmm, local_hhmm,
        cfg.lockout_enabled, cfg.lockout_start_min, cfg.lockout_end_min,
        start_hhmm, end_hhmm,
        g_lockout_active, lockout_calc_local ? "true" : "false",
        lockout_calc_utc ? "true" : "false",
        cfg.conn_mode, g_time_valid,
        sntp_enabled ? "true" : "false", sntp_status_name(sync_status),
        server ? server : "",
        (unsigned long)esp_sntp_get_sync_interval(), esp_sntp_getreachability(0),
        (long long)g_last_time_sync_epoch, last_sync_s,
        (long long)since_sync_s, (unsigned long)g_time_sync_count);
    if (n < 0) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Clock format failed");
    if (n >= (int)sizeof(json)) n = strlen(json);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

static esp_err_t setup_clock_resync_post(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);
    bool ok = time_sync_request_resync();
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, ok ? "{\"resync_requested\":true}" : "{\"resync_requested\":false}");
}

static esp_err_t setup_root_get(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);
    static const char html[] =
        "<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Tank Setup</title><style>"
        "body{font-family:system-ui,Segoe UI,sans-serif;margin:0;background:#f5f7f8;color:#172026}"
        "main{max-width:760px;margin:auto;padding:18px}section{background:white;border:1px solid #d8e0e4;border-radius:8px;padding:14px;margin:12px 0}"
        "h1{font-size:24px;margin:4px 0 12px}h2{font-size:17px;margin:0 0 10px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
        ".v{padding:10px;background:#eef3f5;border-radius:6px}label{display:block;font-size:13px;margin:10px 0 3px}input,select,button{font:inherit;padding:10px;border-radius:6px;border:1px solid #b8c4ca;box-sizing:border-box;width:100%;color:#172026;background:#fff}"
        "button{background:#0b6f85;color:white;border:0;margin-top:10px}.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}.warn{color:#9a4b00}.ok{color:#157347}.hint{font-size:13px;color:#5a6870;margin:8px 0}.bad{color:#b42318}"
        "</style></head><body><main><h1>Tank Controller Setup</h1>"
        "<section><h2>Live Sensor Check</h2><div class=grid>"
        "<div class=v>Baro <b id=baro>-</b> hPa</div><div class=v>Tank <b id=tank>-</b> hPa</div>"
        "<div class=v>Air temp <b id=airtemp>-</b> C</div><div class=v>Water temp <b id=watertemp>-</b> C</div>"
        "<div class=v>Depth <b id=depth>-</b> cm</div><div class=v>Low alert <b id=lowalert>-</b></div>"
        "<div class=v>Relay <b id=relay>-</b></div><div class=v>Fault <b id=fault>-</b></div>"
        "<div class=v>Lockout <b id=lockstat>-</b></div><div class=v>Clock <b id=clockstat>-</b></div>"
        "<div class=v>WiFi <b id=wifistat>-</b></div><div class=v>MQTT <b id=mqttstat>-</b></div>"
        "</div><p id=cal></p></section>"
        "<section><h2>Clock / Lockout Time</h2><div class=grid>"
        "<div class=v>Current time <b id=localtime>-</b></div><div class=v>SNTP <b id=sntpstat>-</b></div>"
        "<div class=v>Lockout window <b id=lockwindow>-</b></div><div class=v>Now in window <b id=lockcalc>-</b></div>"
        "</div><button onclick=resetTime()>Reset time</button><p id=clockmsg class=hint></p></section>"
        "<section><h2>1. Open-Air Test</h2><p>Keep both sensors in open air, then save the offset.</p><button onclick=\"calPost('/api/open_air','This will replace the saved open-air offset. Continue?')\">Run open-air test</button></section>"
        "<section><h2>2. Full Tank Calibration</h2><p>Lower the tank sensor into a full tank, keep the baro sensor in air, then save full.</p><button onclick=\"calPost('/api/full_tank','This will replace full-tank calibration and reset level setpoints. Continue?')\">Set full tank</button><p id=calmsg></p></section>"
        "<section><h2>Operating Parameters</h2><p class=hint>Use: Low alert <= Operating level < Full level <= Tank height.</p><div class=row>"
        "<label>Low alert cm<input id=low type=number></label><label>Operating level cm<input id=op type=number></label>"
        "<label>Full level cm<input id=full type=number></label>"
        "<label>Tank height cm<input id=height type=number></label><label>Density kg/m3<input id=density type=number></label>"
        "<label>Mode<select id=mode><option value=0>auto</option><option value=1>force on</option><option value=2>force off</option></select></label>"
        "<label id=connfield>Connectivity<select id=conn><option value=255>choose</option><option value=0>standalone</option><option value=1>zigbee</option><option value=2>local wifi</option></select></label>"
        "<label>Pump lockout<select id=lock><option value=0>off</option><option value=1>on</option></select></label>"
        "</div><p id=lockhint class=hint></p><div id=locktimes class=row>"
        "<label>Lockout start<input id=lstart type=time></label><label>Lockout end<input id=lend type=time></label>"
        "</div></section><section id=wifisec><h2>Local WiFi / MQTT</h2><div class=row>"
        "<label>WiFi SSID<input id=ssid></label><label>WiFi password<input id=wpass type=password autocomplete=off></label>"
        "<label>MQTT broker IP/host<input id=mh placeholder='192.168.1.10'></label><label>MQTT topic<input id=mt></label>"
        "<label>MQTT user<input id=mu></label><label>MQTT password<input id=mp type=password autocomplete=off></label>"
        "</div></section>"
        "<section><h2>Page Security</h2><p id=webhint class=hint></p>"
        "<label>Page login password (user: admin)<input id=webpass type=password autocomplete=off placeholder='set a new password'></label></section>"
        "<section><h2>Save</h2><p id=cal2 class=warn></p><p id=connhint class=warn></p><p id=rulehint class=hint></p>"
        "<button onclick=save()>Save parameters</button> <button onclick=saveReboot()>Save &amp; reboot</button></section>"
        "<p id=msg></p><p class=hint>BOOT: single press changes mode, double press reopens setup, triple press factory-resets.</p></main><script>"
        "const $=id=>document.getElementById(id);"
        "let changed=new Set();let formInit=false;let calibrated=false;let connLocked=false;"
        "function pad(n){return String(n).padStart(2,'0')}function minToTime(m){m=Number(m)||0;return pad(Math.floor(m/60))+':'+pad(m%60)}function timeToMin(v){let p=(v||'00:00').split(':');return (Number(p[0])||0)*60+(Number(p[1])||0)}"
        "function updateHints(){var c=$('conn').value;var lock=$('lock').value;$('connfield').style.display=connLocked?'none':'';$('wifisec').style.display=(c=='2')?'':'none';$('locktimes').style.display=(c=='2'&&lock=='1')?'':'none';"
        "$('lockhint').textContent=(c=='1')?'Zigbee mode: Home Assistant controls pump lockout.':(c=='2')?'Local WiFi mode: lockout uses the schedule below.':'Standalone mode: lockout is a manual pump inhibit.';"
        "$('connhint').textContent=connLocked?'Connectivity is locked to '+(c=='2'?'local WiFi':c=='1'?'Zigbee':'standalone')+'. Factory reset is required to change it.':(c=='1')?'Zigbee selected: AP will disappear and the device will join Zigbee.':(c=='2')?'Local WiFi selected: AP will disappear; setup moves to the local WiFi address.':(c=='0')?'Standalone selected: AP will disappear; double-press BOOT to reopen setup.':'Choose a connectivity option, then Save & reboot.';"
        "validateLocal()}"
        "function validateLocal(){let lo=Number($('low').value),op=Number($('op').value),fu=Number($('full').value),hi=Number($('height').value);let ok=lo>=0&&op>=lo&&fu>op&&fu<=hi;$('rulehint').textContent=ok?'Level order looks valid.':'Check level order: low <= operating < full <= tank height.';$('rulehint').className=ok?'hint':'hint bad';return ok}"
        "function bindEdits(){['low','op','full','height','density','mode','conn','lock','lstart','lend','ssid','wpass','mh','mu','mp','mt','webpass'].forEach(id=>{$(id).oninput=()=>{changed.add(id);updateHints()};$(id).onchange=()=>{changed.add(id);updateHints()}})}"
        "function syncForm(s){$('low').value=s.low_cm;$('op').value=s.operating_cm;$('full').value=s.full_cm;$('height').value=s.tank_height_cm;$('density').value=s.density;$('mode').value=s.mode;$('conn').value=s.conn_mode;connLocked=s.conn_mode!=255;$('lock').value=s.lockout_enabled;$('lstart').value=minToTime(s.lockout_start_min);$('lend').value=minToTime(s.lockout_end_min);$('ssid').value=s.wifi_ssid;$('mh').value=s.mqtt_host;$('mu').value=s.mqtt_user;$('mt').value=s.mqtt_topic;if(!$('wpass').value)$('wpass').placeholder='saved/blank';if(!$('mp').value)$('mp').placeholder='saved/blank';updateHints();changed.clear()}"
        "async function refresh(){let s=await(await fetch('/api/status')).json();syncForm(s)}"
        "async function loadClock(){try{let c=await(await fetch('/api/clock')).json();$('localtime').textContent=c.valid?c.local.replace('T',' '):'not synced';$('sntpstat').textContent=c.sntp_status+' #'+c.sync_count;$('lockwindow').textContent=c.lockout_start_hhmm+' - '+c.lockout_end_hhmm;$('lockcalc').textContent=c.lockout_calc_local?'yes':'no';}catch(e){$('localtime').textContent='unavailable';$('sntpstat').textContent='error'}}"
        "async function resetTime(){let r=await fetch('/api/clock_resync',{method:'POST'});$('clockmsg').textContent=r.ok?'Time resync requested.':('Time resync failed: '+await r.text());setTimeout(loadClock,3000)}"
        "async function load(){let r=await fetch('/api/status');let s=await r.json();"
        "$('baro').textContent=s.baro_hpa.toFixed(2);$('tank').textContent=s.tank_hpa.toFixed(2);$('depth').textContent=s.depth_cm;$('fault').textContent=s.fault;"
        "$('airtemp').textContent=s.air_temp_c.toFixed(2);$('watertemp').textContent=s.water_temp_c.toFixed(2);"
        "$('lowalert').textContent=s.low_alert?'ON':'OFF';$('relay').textContent=s.relay?'ON':'OFF';"
        "$('lockstat').textContent=s.lockout_active?'ON':'off';$('clockstat').textContent=s.time_valid?'synced':'not synced';"
        "$('wifistat').textContent=s.wifi_connected?'connected':'off';$('mqttstat').textContent=s.mqtt_connected?'connected':'off';"
        "calibrated=!!s.calibrated;let cal=calibrated?'<span class=ok>Calibrated</span>':'<span class=warn>Not calibrated: AUTO pump control stays safe/off</span>';$('cal').innerHTML=cal;$('cal2').innerHTML=calibrated?'':cal;"
        "$('webhint').textContent=s.web_pass_changed?'Page login required over local WiFi (user: admin).':('Default login admin / " SETUP_WEB_DEFAULT_PASS " in use - set a new password below'+(s.wifi_auth?', required before saving.':' before switching to WiFi.'));$('webhint').className=s.web_pass_changed?'hint':'hint bad';"
        "loadClock();if(!formInit){syncForm(s);formInit=true}}"
        "async function post(u){let r=await fetch(u,{method:'POST'});$('calmsg').textContent=await r.text();$('calmsg').className=r.ok?'ok':'bad';refresh()}"
        "async function calPost(u,msg){if(calibrated&&!confirm(msg))return;await post(u)}"
        "async function save(){let p=new URLSearchParams();changed.forEach(id=>{let v=$(id).value;if(id=='lstart'||id=='lend')v=timeToMin(v);p.append(id,v)});let r=await fetch('/api/config',{method:'POST',body:p});$('msg').textContent=await r.text();if(r.ok){$('wpass').value='';$('mp').value='';$('webpass').value='';refresh()}return r.ok}"
        "async function saveReboot(){let mode=$('conn').value;let ok=await save();if(ok){await fetch('/api/reboot',{method:'POST'});$('msg').textContent=(mode=='2')?'Saved - rebooting; this AP will disappear and the page will move to local WiFi.':(mode=='1')?'Saved - rebooting; this AP will disappear and the device will join Zigbee.':'Saved - rebooting; this AP will disappear. Double-press BOOT to reopen setup.'}}"
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
    WEB_REQUIRE_AUTH(req);
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
    WEB_REQUIRE_AUTH(req);
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
    WEB_REQUIRE_AUTH(req);
    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg_t cfg = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    uint8_t saved_conn_mode = cfg.conn_mode;
    char form[900] = "";
    if (req->content_len > 0) {
        esp_err_t err = read_request_body(req, form, sizeof(form));
        if (err == ESP_ERR_INVALID_SIZE) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Form body too large");
        }
        if (err != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing form body");
        }
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
    cfg.lockout_enabled = form_int(form, "lock", cfg.lockout_enabled) ? 1 : 0;
    cfg.lockout_start_min = form_int(form, "lstart", cfg.lockout_start_min);
    cfg.lockout_end_min = form_int(form, "lend", cfg.lockout_end_min);
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

    char newweb[33] = "";
    copy_form_str(form, "webpass", newweb, sizeof(newweb));
    if (newweb[0]) {
        strlcpy(cfg.web_pass, newweb, sizeof(cfg.web_pass));
        cfg.web_pass_changed = 1;
    }
    /* over local WiFi, force the default page password to be changed before any
     * settings can be saved (the setup AP is exempt - you physically join it) */
    if (s_web_auth && !cfg.web_pass_changed) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
            "Set a new page password (admin login) before saving over WiFi");
    }
    if (saved_conn_mode != CONN_UNSET && cfg.conn_mode != saved_conn_mode) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
            "Connectivity mode is locked; factory reset is required to change it");
    }

    const char *bad = cfg_validate(&cfg, true);
    if (bad) {
        char msg[80];
        snprintf(msg, sizeof(msg), "Invalid parameter range: %s", bad);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg);
    }

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    g_cfg = cfg;
    xSemaphoreGive(g_cfg_mtx);
    cfg_save();
    zb_push_mode(cfg.mode);
    ESP_LOGI(TAG, "setup save: lowAlert=%d operating=%d full=%d conn=%u mode=%u lockout=%u %u-%u",
             cfg.low_cm, cfg.operating_cm, cfg.full_cm, cfg.conn_mode, cfg.mode,
             cfg.lockout_enabled, cfg.lockout_start_min, cfg.lockout_end_min);
    return httpd_resp_sendstr(req, "Parameters saved");
}

static void reboot_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(800));   /* let the HTTP response flush first */
    esp_restart();
}

static esp_err_t setup_reboot_post(httpd_req_t *req)
{
    WEB_REQUIRE_AUTH(req);
    httpd_resp_sendstr(req, "Rebooting");
    ESP_LOGI(TAG, "reboot requested via setup portal");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

/* Advertise http://tank.local on whatever interface is up (STA or AP) so the
 * setup page is reachable without hunting for the DHCP address. */
static void mdns_start(void)
{
    static bool started = false;
    if (started) return;
    if (mdns_init() != ESP_OK) { ESP_LOGW(TAG, "mDNS init failed"); return; }
    mdns_hostname_set("tank");
    mdns_instance_name_set("Tank Controller");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    started = true;
    ESP_LOGI(TAG, "mDNS up: http://tank.local");
}

static void web_server_start(void)
{
    if (s_httpd) return;
    httpd_config_t http_cfg = HTTPD_DEFAULT_CONFIG();
    http_cfg.lru_purge_enable = true;
    http_cfg.stack_size = 8192;
    ESP_ERROR_CHECK(httpd_start(&s_httpd, &http_cfg));
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/", .method=HTTP_GET, .handler=setup_root_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/status", .method=HTTP_GET, .handler=setup_status_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/raw_sensor", .method=HTTP_GET, .handler=setup_raw_sensor_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/clock", .method=HTTP_GET, .handler=setup_clock_get });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/clock_resync", .method=HTTP_POST, .handler=setup_clock_resync_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/open_air", .method=HTTP_POST, .handler=setup_open_air_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/full_tank", .method=HTTP_POST, .handler=setup_full_tank_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/config", .method=HTTP_POST, .handler=setup_config_post });
    httpd_register_uri_handler(s_httpd, &(httpd_uri_t){ .uri="/api/reboot", .method=HTTP_POST, .handler=setup_reboot_post });
    mdns_start();
}

static void time_sync_configure(void)
{
    setenv("TZ", LOCAL_TIMEZONE_POSIX, 1);
    tzset();
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_set_sync_interval(SNTP_SYNC_INTERVAL_MS);
}

static void time_sync_start(void)
{
    time_sync_configure();
    if (esp_sntp_enabled()) return;
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    ESP_LOGI(TAG, "SNTP time sync started: server=pool.ntp.org tz=%s interval=%lums",
             LOCAL_TIMEZONE_POSIX, (unsigned long)SNTP_SYNC_INTERVAL_MS);
}

static bool time_sync_request_resync(void)
{
    time_sync_start();
    if (!esp_sntp_enabled()) return false;
    return esp_sntp_restart();
}

static void setup_portal_start(void)
{
    if (s_setup_portal_active) {
        ESP_LOGI(TAG, "setup AP already active");
        return;
    }
    if (s_wifi_started) {
        ESP_LOGW(TAG, "setup AP request ignored: WiFi station is already running");
        return;
    }

    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    s_wifi_started = true;

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
    s_setup_portal_active = true;

    ESP_LOGI(TAG, "setup AP started: SSID=%s URL=http://192.168.4.1", ap_cfg.ap.ssid);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)event_data;
        /* reason codes: 15=4WAY_HANDSHAKE_TIMEOUT (bad password), 2=AUTH_EXPIRE,
         * 201=NO_AP_FOUND, 200/205=beacon/connection loss (weak signal) */
        if (s_wifi_retry++ < WIFI_MAX_RETRY) {
            ESP_LOGW(TAG, "WiFi disconnected (reason %d), retry %d/%d", d->reason, s_wifi_retry, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "WiFi connect failed (reason %d)", d->reason);
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
    s_wifi_started = true;
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
        s_wifi_started = false;
        return false;
    }

    time_sync_start();
    s_web_auth = true;            /* on shared WiFi: require admin login for the page */
    web_server_start();
    mqtt_start();
    return true;
}

#endif

/* Full factory reset: wipe our saved config (-> conn_mode UNSET, so it boots into
 * the setup AP) AND the Zigbee network, then reboot as a brand-new device.
 * Works in any mode (WiFi/Zigbee/setup). */
static void factory_reset(void)
{
    ESP_LOGW(TAG, "FACTORY RESET: clearing config + Zigbee network");
    relay_set(false);

    nvs_handle_t h;
    if (nvs_open("tank", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }

    if (s_zb_started) {
        esp_zb_factory_reset();          /* erases Zigbee NVRAM and reboots */
    } else {
        /* Zigbee stack not running (WiFi/setup mode): erase its NVRAM directly */
        const esp_partition_t *p;
        p = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "zb_storage");
        if (p) esp_partition_erase_range(p, 0, p->size);
        p = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "zb_fct");
        if (p) esp_partition_erase_range(p, 0, p->size);
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }
}

/* ----------------------------- button task ------------------------------ */
/* Runs in all modes. Actions happen only after release, so a factory reset
 * cannot reboot while the BOOT strapping pin is held low. */
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

    /* A BOOT hold during reset selects the ROM downloader. If application code
     * does start with the pin low, ignore it until it has been released. */
    while (gpio_get_level(BUTTON_GPIO) == 0) vTaskDelay(pdMS_TO_TICKS(20));

    bool prev_down = false;
    int64_t press_us = 0;
    int64_t last_release_us = 0;
    uint8_t click_count = 0;

    while (1) {
        bool down = (gpio_get_level(BUTTON_GPIO) == 0);   /* active low */
        int64_t now = esp_timer_get_time();

        if (down && !prev_down) {
            press_us = now;
        } else if (!down && prev_down) {
            int64_t held_us = now - press_us;
            if (held_us >= (int64_t)BUTTON_DEBOUNCE_MS * 1000 &&
                held_us <= (int64_t)BUTTON_CLICK_MAX_MS * 1000) {
                click_count++;
                last_release_us = now;

                if (click_count >= 3) {
                    click_count = 0;
                    ESP_LOGW(TAG, "button triple-click -> full factory reset");
                    factory_reset();             /* BOOT is released before reboot */
                }
            } else {
                click_count = 0;                 /* ignore bounce and long holds */
            }
        } else if (!down && click_count > 0 &&
                   (now - last_release_us) >= (int64_t)BUTTON_MULTI_CLICK_MS * 1000) {
            uint8_t clicks = click_count;
            click_count = 0;

            if (clicks == 2) {
#if SETUP_PORTAL_ENABLE
                ESP_LOGW(TAG, "button double-click -> temporary setup AP");
                setup_portal_start();
#else
                ESP_LOGW(TAG, "button double-click ignored: setup portal disabled");
#endif
            } else {
                uint8_t m;
                xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
                g_cfg.mode = (g_cfg.mode + 1) % 3;   /* AUTO/ON/OFF */
                m = g_cfg.mode;
                xSemaphoreGive(g_cfg_mtx);
                cfg_save();
                zb_push_mode(m);
                ESP_LOGI(TAG, "button single-click -> mode %u (0=auto 1=on 2=off)", m);
            }
        }

        prev_down = down;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------------------------- control task ------------------------------ */
static void control_task(void *arg)
{
    ESP_ERROR_CHECK(lps2x_bus_init(&g_sensor_bus));

    /* one-shot I2C scan to help confirm wiring / actual sensor addresses */
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d ...", I2C_SDA_GPIO, I2C_SCL_GPIO);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(g_sensor_bus, a, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  found I2C device @ 0x%02x", a);
            found++;
        }
    }
    if (found == 0) ESP_LOGW(TAG, "  no I2C devices found (sensors connected/powered?)");

    xSemaphoreTake(g_sensor_mtx, portMAX_DELAY);
    g_baro_sensor_ok = (lps2x_init(g_sensor_bus, LPS_BARO_ADDR, &g_baro_sensor) == ESP_OK);
    g_tank_sensor_ok = (lps2x_init(g_sensor_bus, LPS_TANK_ADDR, &g_tank_sensor) == ESP_OK);
    bool baro_ok = g_baro_sensor_ok;
    bool tank_ok = g_tank_sensor_ok;
    xSemaphoreGive(g_sensor_mtx);
    ESP_LOGI(TAG, "sensors: baro(0x%02x)=%s  tank(0x%02x)=%s", LPS_BARO_ADDR,
             baro_ok ? "OK" : "absent", LPS_TANK_ADDR, tank_ok ? "OK" : "absent");
    if (!baro_ok && !tank_ok) ESP_LOGW(TAG, "no sensors - relay stays in failsafe OFF");
    else if (!baro_ok || !tank_ok) ESP_LOGW(TAG, "single sensor only - level not measurable (diagnostic)");

    int fault_count = 0;
    int64_t last_off_us = 0, last_on_us = 0;
    float last_p_tank = 0, last_p_baro = 0;
    float last_t_tank = 0, last_t_baro = 0;
    bool have_last_tank = false, have_last_baro = false;
    int16_t last_depth_cm = 0;
    uint8_t last_level_pct = 0;
    bool have_last_level = false;
    bool full_inhibit = false;
    int full_clear_count = 0;

    while (1) {
        cfg_t cfg;
        xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
        cfg = g_cfg;
        xSemaphoreGive(g_cfg_mtx);

        /* hot-plug: a sensor not present at boot (e.g. the submerged probe on a
         * long cable connected after power-up) gets picked up here so it works
         * in WiFi/Zigbee mode too, not just when present during the boot scan. */
        xSemaphoreTake(g_sensor_mtx, portMAX_DELAY);
        if (!g_baro_sensor_ok && lps2x_init(g_sensor_bus, LPS_BARO_ADDR, &g_baro_sensor) == ESP_OK) {
            g_baro_sensor_ok = true;
            ESP_LOGI(TAG, "baro sensor (0x%02x) appeared", LPS_BARO_ADDR);
        }
        if (!g_tank_sensor_ok && lps2x_init(g_sensor_bus, LPS_TANK_ADDR, &g_tank_sensor) == ESP_OK) {
            g_tank_sensor_ok = true;
            ESP_LOGI(TAG, "tank sensor (0x%02x) appeared", LPS_TANK_ADDR);
        }
        baro_ok = g_baro_sensor_ok;
        tank_ok = g_tank_sensor_ok;

        /* read whichever sensors are present (each independent), taking the
         * median of a few one-shot conversions so a single bad zero sample
         * cannot look like an empty tank. */
        float p_tank = 0, p_baro = 0;
        float t_tank = 0, t_baro = 0;
        float p_tank_samples[SENSOR_AVG_SAMPLES];
        float p_baro_samples[SENSOR_AVG_SAMPLES];
        float t_tank_samples[SENSOR_AVG_SAMPLES];
        float t_baro_samples[SENSOR_AVG_SAMPLES];
        int baro_n = 0, tank_n = 0;
        int baro_fresh_n = 0, tank_fresh_n = 0;
        for (int i = 0; i < SENSOR_AVG_SAMPLES; i++) {
            float p = 0;
            float t = 0;
            if (baro_ok && lps2x_read(&g_baro_sensor, &p, &t) == ESP_OK) {
                if (sensor_pressure_valid(p)) {
                    p_baro_samples[baro_n] = p;
                    t_baro_samples[baro_n] = t;
                    baro_n++;
                    baro_fresh_n++;
                } else if (have_last_baro) {
                    p_baro_samples[baro_n] = last_p_baro;
                    t_baro_samples[baro_n] = last_t_baro;
                    baro_n++;
                }
            } else if (baro_ok && have_last_baro) {
                p_baro_samples[baro_n] = last_p_baro;
                t_baro_samples[baro_n] = last_t_baro;
                baro_n++;
            }
            if (tank_ok && lps2x_read(&g_tank_sensor, &p, &t) == ESP_OK) {
                if (sensor_pressure_valid(p)) {
                    p_tank_samples[tank_n] = p;
                    t_tank_samples[tank_n] = t;
                    tank_n++;
                    tank_fresh_n++;
                } else if (have_last_tank) {
                    p_tank_samples[tank_n] = last_p_tank;
                    t_tank_samples[tank_n] = last_t_tank;
                    tank_n++;
                }
            } else if (tank_ok && have_last_tank) {
                p_tank_samples[tank_n] = last_p_tank;
                t_tank_samples[tank_n] = last_t_tank;
                tank_n++;
            }
            if (i + 1 < SENSOR_AVG_SAMPLES) vTaskDelay(pdMS_TO_TICKS(SENSOR_AVG_DELAY_MS));
        }
        xSemaphoreGive(g_sensor_mtx);
        bool baro_fresh = baro_fresh_n > 0;
        bool tank_fresh = tank_fresh_n > 0;
        if (baro_n > 0) {
            p_baro = median_float(p_baro_samples, baro_n);
            t_baro = median_float(t_baro_samples, baro_n);
            if (baro_fresh) {
                last_p_baro = p_baro;
                last_t_baro = t_baro;
                have_last_baro = true;
            }
        } else if (have_last_baro) {
            p_baro = last_p_baro;
            t_baro = last_t_baro;
        }
        if (tank_n > 0) {
            p_tank = median_float(p_tank_samples, tank_n);
            t_tank = median_float(t_tank_samples, tank_n);
            if (tank_fresh) {
                last_p_tank = p_tank;
                last_t_tank = t_tank;
                have_last_tank = true;
            }
        } else if (have_last_tank) {
            p_tank = last_p_tank;
            t_tank = last_t_tank;
        }
        bool baro_rd = baro_n > 0 || have_last_baro;
        bool tank_rd = tank_n > 0 || have_last_tank;
        bool level_fresh = baro_fresh && tank_fresh;
        g_baro_pressure_hpa = baro_rd ? (int16_t)lroundf(p_baro) : -1;
        g_tank_pressure_hpa = tank_rd ? (int16_t)lroundf(p_tank) : -1;
        g_baro_pressure_hpa_x100 = baro_rd ? (int32_t)lroundf(p_baro * 100.0f) : -1;
        g_tank_pressure_hpa_x100 = tank_rd ? (int32_t)lroundf(p_tank * 100.0f) : -1;
        g_air_temp_c_x100 = baro_rd ? (int16_t)lroundf(t_baro * 100.0f) : INT16_MIN;
        g_water_temp_c_x100 = tank_rd ? (int16_t)lroundf(t_tank * 100.0f) : INT16_MIN;
        bool level_ok = false;     /* true only when a real water level is known */

        if (baro_rd && tank_rd) {
            bool sensor_miss_fault = false;
            if (level_fresh) {
                fault_count = 0;
            } else if (++fault_count >= SENSOR_FAULT_LIMIT) {
                sensor_miss_fault = true;
            }

            /* full hydrostatic measurement: hPa -> Pa -> depth */
            float corrected_delta_hpa = (p_tank - p_baro) - ((float)cfg.air_offset_hpa_x100 / 100.0f);
            float head_pa = corrected_delta_hpa * 100.0f;
            float depth_cm = head_pa / ((float)cfg.density * GRAVITY_MS2) * 100.0f + SENSOR_OFFSET_CM;
            if (depth_cm < 0) depth_cm = 0;
            int16_t measured_depth_cm = (int16_t)lroundf(depth_cm);
            int16_t span = cfg.tank_h_cm > 0 ? cfg.tank_h_cm : 1;
            int pct = (int)lroundf(depth_cm * 100.0f / span);
            uint8_t measured_level_pct = pct < 0 ? 0 : (pct > 100 ? 100 : pct);
            bool sensor_drop_fault = false;
            if (level_fresh && have_last_level &&
                measured_depth_cm + SENSOR_MAX_DROP_CM < last_depth_cm) {
                sensor_drop_fault = true;
                g_depth_cm = last_depth_cm;
                g_level_pct = last_level_pct;
            } else {
                g_depth_cm = measured_depth_cm;
                g_level_pct = measured_level_pct;
            }
            uint8_t next_fault = 0;
            bool measured_level_ok = true;
#if BENCH_REQUIRE_EQUAL_PRESSURE
            if (fabsf(p_tank - p_baro) > BENCH_MAX_DELTA_HPA) {
                next_fault = 4;     /* 4 = bench sensors disagree */
                measured_level_ok = false;
            }
#endif
            if (measured_level_ok && !cfg.calibrated) {
                next_fault = 5;     /* 5 = setup/calibration incomplete */
                measured_level_ok = false;
            }
            if (measured_level_ok && g_depth_cm > cfg.tank_h_cm + MAX_DEPTH_OVER_TANK_CM) {
                next_fault = 3;     /* 3 = pressure delta is implausible */
                measured_level_ok = false;
            }
            if (sensor_miss_fault || sensor_drop_fault) {
                g_fault = 1;        /* 1 = repeated missed/invalid sensor reads */
                level_ok = false;
            } else {
                g_fault = next_fault;
                level_ok = measured_level_ok;
            }
            if (level_ok && level_fresh) {
                last_depth_cm = g_depth_cm;
                last_level_pct = g_level_pct;
                have_last_level = true;
            }
            g_low_alert = (level_ok && g_depth_cm <= cfg.low_cm) ? 1 : 0;
#if LOG_SENSOR_READINGS
            ESP_LOGI(TAG, "sensor read: baro=%.2fhPa %.2fC(%d/%d%s) tank=%.2fhPa %.2fC(%d/%d%s) depth=%dcm level=%u%% fault=%u%s",
                     p_baro, t_baro, baro_fresh_n, SENSOR_AVG_SAMPLES,
                     baro_fresh_n == SENSOR_AVG_SAMPLES ? "" : " held",
                     p_tank, t_tank, tank_fresh_n, SENSOR_AVG_SAMPLES,
                     tank_fresh_n == SENSOR_AVG_SAMPLES ? "" : " held",
                     g_depth_cm, g_level_pct, g_fault,
                     sensor_drop_fault ? " sudden-drop" : "");
#endif
        } else if (baro_rd || tank_rd) {
            /* DIAGNOSTIC: only one sensor present - cannot measure level.
             * Report that sensor's absolute pressure (hPa, ~1013 at sea level)
             * in 'depth' so it's visible over Zigbee = "this sensor is alive".
             * Relay stays in failsafe (no real level). */
            bool any_fresh = baro_fresh || tank_fresh;
            g_depth_cm = (int16_t)lroundf(baro_rd ? p_baro : p_tank);
            g_level_pct = 0;
            if (any_fresh) {
                fault_count = 0;
                g_fault = 2;        /* 2 = single-sensor diagnostic mode */
            } else if (++fault_count >= SENSOR_FAULT_LIMIT) {
                g_fault = 1;        /* stale held diagnostic value */
            } else {
                g_fault = 2;
            }
            g_low_alert = 0;
#if LOG_SENSOR_READINGS
            ESP_LOGW(TAG, "sensor read: only %s OK%s, pressure=%.2fhPa temp=%.2fC",
                     baro_rd ? "baro" : "tank", any_fresh ? "" : " (held)",
                     baro_rd ? p_baro : p_tank,
                     baro_rd ? t_baro : t_tank);
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

        bool at_or_above_full = baro_rd && tank_rd &&
            (g_depth_cm >= cfg.full_cm || g_level_pct >= 100);
        if (at_or_above_full) {
            if (!full_inhibit) {
                ESP_LOGW(TAG, "full inhibit -> ON (depth=%dcm level=%u%% full=%dcm); pump held off until %d fresh below-full reads",
                         g_depth_cm, g_level_pct, cfg.full_cm, FULL_INHIBIT_CLEAR_READINGS);
            }
            full_inhibit = true;
            full_clear_count = 0;
        } else if (full_inhibit) {
            if (level_ok && level_fresh && g_depth_cm < cfg.full_cm && g_level_pct < 100) {
                if (full_clear_count < FULL_INHIBIT_CLEAR_READINGS) full_clear_count++;
                if (full_clear_count >= FULL_INHIBIT_CLEAR_READINGS) {
                    full_inhibit = false;
                    full_clear_count = 0;
                    ESP_LOGI(TAG, "full inhibit -> OFF after %d fresh below-full reads",
                             FULL_INHIBIT_CLEAR_READINGS);
                }
            } else {
                full_clear_count = 0;
            }
        }

        if (at_or_above_full && cfg.mode == MODE_ON) {
            bool mode_changed = false;
            xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
            if (g_cfg.mode == MODE_ON) {
                g_cfg.mode = MODE_AUTO;
                mode_changed = true;
            }
            xSemaphoreGive(g_cfg_mtx);
            if (mode_changed) {
                cfg.mode = MODE_AUTO;
                cfg_save();
                zb_push_mode(MODE_AUTO);
                ESP_LOGW(TAG, "force-on reverted to AUTO at full level (depth=%dcm level=%u%% full=%dcm)",
                         g_depth_cm, g_level_pct, cfg.full_cm);
            }
        }

        /* ---- decide relay state ---- */
        int64_t now = esp_timer_get_time();
        bool want = g_relay_on;

        if (cfg.mode == MODE_ON) {
            want = level_ok && level_fresh && !at_or_above_full;
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

        bool locked_out = lockout_is_active(&cfg);

        /* anti-short-cycle guards (only when actually pump-controlling) */
        if (cfg.mode == MODE_AUTO && level_ok) {
            if (want && !g_relay_on) {
                if (now - last_off_us < (int64_t)PUMP_MIN_OFF_MS * 1000) want = false;
            }
            if (!want && g_relay_on && PUMP_MIN_ON_MS > 0) {
                if (now - last_on_us < (int64_t)PUMP_MIN_ON_MS * 1000) want = true;
            }
        }

        if (cfg.mode == MODE_AUTO && locked_out) {
            want = false;
        }
        if (full_inhibit) {
            want = false;
        }

        if (want != g_relay_on) {
            relay_set(want);
            if (want) last_on_us = now; else last_off_us = now;
            ESP_LOGI(TAG, "relay -> %s (depth=%dcm %d%% mode=%d fault=%d lockout=%u full_inhibit=%u)",
                     want ? "ON" : "OFF", g_depth_cm, g_level_pct, cfg.mode, g_fault,
                     g_lockout_active, full_inhibit ? 1 : 0);
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
static uint32_t s_ota_rx_bytes = 0;
static uint32_t s_ota_skip  = 0;            /* sub-element header bytes to drop  */
static volatile bool s_ota_finish = false;
static volatile bool s_ota_failed = false;
/* s_ota_active is declared up top so the telemetry gate can see it */
#define OTA_SUBELEMENT_HDR  6               /* tag id (2) + length (4)           */

/* Writer task: does all flash work off the radio callback and batches the
 * small Zigbee blocks. Writing every block separately eventually starves the
 * C5 radio long enough for ZBOSS to assert in its MAC ACK timeout handler. */
static void ota_writer_task(void *arg)
{
    ESP_LOGI(TAG, "OTA writer -> %s", s_ota_part->label);

    uint8_t *buf = malloc(4096);
    if (!buf) {
        ESP_LOGE(TAG, "OTA writer buffer allocation failed");
        s_ota_failed = true;
        s_ota_finish = true;
    }

    size_t used = 0;
    while (1) {
        if (!s_ota_failed) {
            size_t n = xStreamBufferReceive(s_ota_sb, buf + used, 4096 - used,
                                            pdMS_TO_TICKS(200));
            used += n;
            if (used == 4096) {
                if (esp_ota_write(s_ota_handle, buf, used) != ESP_OK) {
                    s_ota_failed = true;
                } else {
                    s_ota_bytes += used;
                    used = 0;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        if (s_ota_finish && (s_ota_failed || xStreamBufferIsEmpty(s_ota_sb))) {
            if (!s_ota_failed && used > 0) {
                if (esp_ota_write(s_ota_handle, buf, used) != ESP_OK) {
                    s_ota_failed = true;
                } else {
                    s_ota_bytes += used;
                }
            }
            break;
        }
    }
    free(buf);

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
    if (msg.info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "OTA callback status 0x%x", msg.info.status);
        return ESP_FAIL;
    }
    switch (msg.upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        if (s_ota_active) break;
        s_ota_bytes = 0;
        s_ota_rx_bytes = 0;
        s_ota_skip = OTA_SUBELEMENT_HDR;
        s_ota_finish = false;
        s_ota_failed = false;
        s_ota_part = esp_ota_get_next_update_partition(NULL);
        esp_err_t err = s_ota_part
            ? esp_ota_begin(s_ota_part, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle)
            : ESP_ERR_NOT_FOUND;
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
            s_ota_failed = true;
            return err;
        }
        if (!s_ota_sb) s_ota_sb = xStreamBufferCreate(16384, 1);
        if (!s_ota_sb) {
            esp_ota_abort(s_ota_handle);
            s_ota_handle = 0;
            s_ota_failed = true;
            return ESP_ERR_NO_MEM;
        }
        xStreamBufferReset(s_ota_sb);
        s_ota_active = true;
        if (xTaskCreate(ota_writer_task, "ota_wr", 8192, NULL, 4, NULL) != pdPASS) {
            esp_ota_abort(s_ota_handle);
            s_ota_handle = 0;
            s_ota_active = false;
            s_ota_failed = true;
            return ESP_ERR_NO_MEM;
        }
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
            if (n) {
                uint32_t before = s_ota_rx_bytes;
                if (xStreamBufferSend(s_ota_sb, p, n, pdMS_TO_TICKS(200)) != n) {
                    ESP_LOGE(TAG, "OTA receive queue full");
                    s_ota_failed = true;
                    return ESP_ERR_NO_MEM;
                }
                s_ota_rx_bytes += n;
                if (before == 0 || (before >> 16) != (s_ota_rx_bytes >> 16)) {
                    ESP_LOGI(TAG, "OTA received %u bytes", (unsigned)s_ota_rx_bytes);
                }
            }
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

static void ota_match_desc_cb(esp_zb_zdp_status_t status, uint16_t addr,
                              uint8_t endpoint, void *user_ctx)
{
    (void)user_ctx;
    if (status != ESP_ZB_ZDP_STATUS_SUCCESS || s_ota_active) {
        ESP_LOGW(TAG, "OTA server discovery failed (status=0x%x)", status);
        return;
    }
    esp_zb_ota_upgrade_client_query_interval_set(ZB_ENDPOINT, OTA_QUERY_INTERVAL_S);
    esp_err_t err = esp_zb_ota_upgrade_client_query_image_req(addr, endpoint);
    ESP_LOGI(TAG, "OTA server 0x%04x ep%u, query -> %s",
             addr, endpoint, esp_err_to_name(err));
}

static void ota_discover_server_cb(uint8_t arg)
{
    (void)arg;
    if (!g_zb_joined || s_ota_active) return;

    static uint16_t clusters[] = {ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
    esp_zb_zdo_match_desc_req_param_t req = {
        .addr_of_interest = 0x0000,
        .dst_nwk_addr = 0x0000,
        .num_in_clusters = 1,
        .num_out_clusters = 0,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_list = clusters,
    };
    esp_err_t err = esp_zb_zdo_match_cluster(&req, ota_match_desc_cb, NULL);
    ESP_LOGI(TAG, "OTA server discovery -> %s", esp_err_to_name(err));
}

/* --------------------------- Zigbee write handler ----------------------- */
static esp_err_t zb_set_attr_cb(const esp_zb_zcl_set_attr_value_message_t *m)
{
    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ANALOG_OUTPUT &&
        m->attribute.id == ESP_ZB_ZCL_ATTR_ANALOG_OUTPUT_PRESENT_VALUE_ID) {
        float raw = *(float *)m->attribute.data.value;
        if (!isfinite(raw)) return ESP_ERR_INVALID_ARG;

        xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
        cfg_t next = g_cfg;
        bool handled = true;
        switch (m->info.dst_endpoint) {
            case ZB_EP_SET_LOW:        next.low_cm = (int16_t)lroundf(raw); break;
            case ZB_EP_SET_OPERATING:  next.operating_cm = (int16_t)lroundf(raw); break;
            case ZB_EP_SET_FULL:       next.full_cm = (int16_t)lroundf(raw); break;
            case ZB_EP_SET_TANK_HEIGHT:next.tank_h_cm = (int16_t)lroundf(raw); break;
            case ZB_EP_SET_DENSITY:    next.density = (uint16_t)lroundf(raw); break;
            case ZB_EP_SET_MODE:       next.mode = (uint8_t)lroundf(raw); break;
            default: handled = false; break;
        }
        const char *bad = handled ? cfg_validate(&next, false) : "unknown_endpoint";
        if (!bad) g_cfg = next;
        cfg_t snap = g_cfg;
        xSemaphoreGive(g_cfg_mtx);

        if (bad) {
            ESP_LOGW(TAG, "Zigbee standard setting rejected (%s): ep=%u value=%.2f",
                     bad, m->info.dst_endpoint, (double)raw);
        } else {
            cfg_save();
            ESP_LOGI(TAG, "Zigbee setting: low=%d operating=%d full=%d tankH=%d rho=%u mode=%u",
                     snap.low_cm, snap.operating_cm, snap.full_cm, snap.tank_h_cm,
                     snap.density, snap.mode);
        }
        return ESP_OK;
    }

    if (m->info.dst_endpoint != ZB_ENDPOINT) return ESP_OK;

    if (m->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        m->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
        uint8_t lockout = *(uint8_t *)m->attribute.data.value ? 1 : 0;
        xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
        g_cfg.lockout_enabled = lockout;
        xSemaphoreGive(g_cfg_mtx);
        cfg_save();
        ESP_LOGI(TAG, "Zigbee AUTO lockout -> %s", lockout ? "ON" : "OFF");
        return ESP_OK;
    }

    if (m->info.cluster != ZB_CUSTOM_CLUSTER_ID) return ESP_OK;

    xSemaphoreTake(g_cfg_mtx, portMAX_DELAY);
    cfg_t next = g_cfg;
    switch (m->attribute.id) {
        case ATTR_SET_LOW_CM:     next.low_cm    = *(int16_t *)m->attribute.data.value; break;
        case ATTR_SET_FULL_CM:    next.full_cm   = *(int16_t *)m->attribute.data.value; break;
        case ATTR_TANK_HEIGHT_CM: next.tank_h_cm = *(int16_t *)m->attribute.data.value; break;
        case ATTR_DENSITY:        next.density   = *(uint16_t *)m->attribute.data.value; break;
        case ATTR_MODE:           next.mode      = *(uint8_t *)m->attribute.data.value; break;
        case ATTR_OPERATING_CM:   next.operating_cm = *(int16_t *)m->attribute.data.value; break;
        case ATTR_LOCKOUT_ENABLED:next.lockout_enabled = *(uint8_t *)m->attribute.data.value ? 1 : 0; break;
        case ATTR_LOCKOUT_START_MIN: next.lockout_start_min = *(uint16_t *)m->attribute.data.value; break;
        case ATTR_LOCKOUT_END_MIN:   next.lockout_end_min = *(uint16_t *)m->attribute.data.value; break;
        default: break;
    }
    const char *bad = cfg_validate(&next, false);
    if (bad) {
        ESP_LOGW(TAG, "Zigbee cfg update rejected (%s): attr=0x%04x", bad, m->attribute.id);
    } else {
        g_cfg = next;
    }
    cfg_t snap = g_cfg;
    xSemaphoreGive(g_cfg_mtx);

    if (!bad) cfg_save();
    ESP_LOGI(TAG, "cfg update: lowAlert=%d operating=%d full=%d tankH=%d rho=%d mode=%d lockout=%u",
             snap.low_cm, snap.operating_cm, snap.full_cm, snap.tank_h_cm, snap.density,
             snap.mode, snap.lockout_enabled);
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
        ESP_LOGI(TAG, "OTA query response: status=%d server=0x%04x ep%u version=0x%08x size=%u",
                 m->query_status, m->server_addr.u.short_addr, m->server_endpoint,
                 (unsigned)m->file_version, (unsigned)m->image_size);
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
    s16 = INT16_MIN;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_AIR_TEMP_CX100, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_WATER_TEMP_CX100, ESP_ZB_ZCL_ATTR_TYPE_S16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s16);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_RELAY_STATE, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &u8);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LOCKOUT_ACTIVE, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &u8);
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_TIME_VALID, ESP_ZB_ZCL_ATTR_TYPE_U8,
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
    u8 = g_cfg.lockout_enabled;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LOCKOUT_ENABLED, ESP_ZB_ZCL_ATTR_TYPE_U8,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &u8);
    u16 = g_cfg.lockout_start_min;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LOCKOUT_START_MIN, ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &u16);
    u16 = g_cfg.lockout_end_min;
    esp_zb_custom_cluster_add_custom_attr(c, ATTR_LOCKOUT_END_MIN, ESP_ZB_ZCL_ATTR_TYPE_U16,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &u16);

    esp_zb_cluster_list_add_custom_cluster(list, c, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
}

/* a single-cluster sensor endpoint carrying a standard genAnalogInput - the
 * stack auto-reports presentValue once the coordinator configures reporting. */
static void add_analog_input_ep(esp_zb_ep_list_t *ep_list, uint8_t endpoint)
{
    esp_zb_analog_input_cluster_cfg_t cfg = { .out_of_service = 0, .present_value = 0, .status_flags = 0 };
    esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&cfg);
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_input_cluster(cl, ai, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t epc = {
        .endpoint = endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epc);
}

/* Standard writable PresentValue endpoint. The endpoint number supplies the
 * tank-specific meaning while the wire protocol remains ordinary Zigbee ZCL. */
static void add_analog_output_ep(esp_zb_ep_list_t *ep_list, uint8_t endpoint, float initial)
{
    esp_zb_analog_output_cluster_cfg_t cfg = {
        .out_of_service = 0, .present_value = initial, .status_flags = 0,
    };
    esp_zb_attribute_list_t *ao = esp_zb_analog_output_cluster_create(&cfg);
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_analog_output_cluster(cl, ao, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t epc = {
        .endpoint = endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epc);
}

/* a single-cluster endpoint carrying a standard msTemperatureMeasurement (degC x100). */
static void add_temp_meas_ep(esp_zb_ep_list_t *ep_list, uint8_t endpoint)
{
    esp_zb_temperature_meas_cluster_cfg_t cfg = { .measured_value = INT16_MIN, .min_value = -4000, .max_value = 12500 };
    esp_zb_attribute_list_t *t = esp_zb_temperature_meas_cluster_create(&cfg);
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_temperature_meas_cluster(cl, t, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_endpoint_config_t epc = {
        .endpoint = endpoint, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID, .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cl, epc);
}

static void esp_zb_task(void *arg)
{
    s_zb_started = true;
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
    };
    esp_zb_attribute_list_t *ota = esp_zb_ota_cluster_create(&ota_cfg);
    esp_zb_zcl_ota_upgrade_client_variable_t ota_var = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_HW_VERSION,
        .max_data_size = OTA_MAX_DATA_SIZE,
    };
    uint16_t ota_server_addr = 0xffff;
    uint8_t ota_server_ep = 0xff;
    esp_zb_ota_cluster_add_attr(ota, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &ota_var);
    esp_zb_ota_cluster_add_attr(ota, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &ota_server_addr);
    esp_zb_ota_cluster_add_attr(ota, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &ota_server_ep);
    esp_zb_cluster_list_add_ota_cluster(clusters, ota, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = ZB_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_OUTPUT_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, clusters, ep_cfg);

    /* standard reportable sensor endpoints (depth/level/temps) - the stack
     * auto-reports these, unlike the custom 0xFC11 cluster */
    add_analog_input_ep(ep_list, ZB_EP_DEPTH);
    add_analog_input_ep(ep_list, ZB_EP_LEVEL);
    add_temp_meas_ep(ep_list, ZB_EP_WATER_TEMP);
    add_temp_meas_ep(ep_list, ZB_EP_AIR_TEMP);
    add_analog_input_ep(ep_list, ZB_EP_FAULT);
    add_analog_input_ep(ep_list, ZB_EP_BARO_PRESSURE);
    add_analog_input_ep(ep_list, ZB_EP_TANK_PRESSURE);
    add_analog_input_ep(ep_list, ZB_EP_LOW_ALERT);
    add_analog_input_ep(ep_list, ZB_EP_RELAY);
    add_analog_input_ep(ep_list, ZB_EP_MODE);
    add_analog_output_ep(ep_list, ZB_EP_SET_LOW, (float)g_cfg.low_cm);
    add_analog_output_ep(ep_list, ZB_EP_SET_OPERATING, (float)g_cfg.operating_cm);
    add_analog_output_ep(ep_list, ZB_EP_SET_FULL, (float)g_cfg.full_cm);
    add_analog_output_ep(ep_list, ZB_EP_SET_TANK_HEIGHT, (float)g_cfg.tank_h_cm);
    add_analog_output_ep(ep_list, ZB_EP_SET_DENSITY, (float)g_cfg.density);
    add_analog_output_ep(ep_list, ZB_EP_SET_MODE, (float)g_cfg.mode);

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
                esp_zb_scheduler_alarm(ota_discover_server_cb, 0, 5000);
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
            esp_zb_scheduler_alarm(ota_discover_server_cb, 0, 5000);
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
    g_sensor_mtx = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((g_cfg_mtx && g_sensor_mtx) ? ESP_OK : ESP_ERR_NO_MEM);
    cfg_load();
    relay_init();                       /* relay OFF before anything else */

    bool setup_mode = false;
    bool wifi_mode = false;
#if SETUP_PORTAL_ENABLE
    setup_mode = (g_cfg.conn_mode == CONN_UNSET);
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
    xTaskCreate(button_task, "button", 3072, NULL, 4, NULL);  /* BOOT reset in any mode */
    if (start_zigbee) {
        xTaskCreate(esp_zb_task, "zigbee", 8192, NULL, 6, NULL);
    }
}
