#include "lps2x.h"
#include "app_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "lps2x";

/* Register map (common to LPS22HB / LPS33HW / LPS35HW) */
#define REG_WHO_AM_I    0x0F
#define REG_CTRL_REG1   0x10
#define REG_CTRL_REG2   0x11
#define REG_STATUS      0x27
#define REG_PRESS_XL    0x28
#define REG_TEMP_L      0x2B

#define WHO_AM_I_VAL    0xB1   /* LPS22HB / LPS33HW / LPS35HW all report 0xB1 */

#define CTRL2_ONE_SHOT  0x01
#define CTRL2_IF_ADD_INC 0x10
#define CTRL2_SWRESET   0x04
#define CTRL1_BDU       0x02
#define STATUS_P_DA     0x01

static esp_err_t wr(lps2x_t *s, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s->dev, buf, 2, 100);
}

static esp_err_t rd(lps2x_t *s, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s->dev, &reg, 1, buf, n, 100);
}

esp_err_t lps2x_bus_init(i2c_master_bus_handle_t *out_bus)
{
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, out_bus);
}

esp_err_t lps2x_init(i2c_master_bus_handle_t bus, uint8_t addr, lps2x_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    out->dev = NULL;
    out->addr = addr;

    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dcfg, &out->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add dev 0x%02x failed: %s", addr, esp_err_to_name(err));
        return err;
    }

    uint8_t id = 0;
    err = rd(out, REG_WHO_AM_I, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02x: no ack", addr);
        goto fail;
    }
    if (id != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "0x%02x: WHO_AM_I=0x%02x (expected 0x%02x)", addr, id, WHO_AM_I_VAL);
        err = ESP_ERR_NOT_FOUND;
        goto fail;
    }

    /* Soft reset, then enable register auto-increment for block reads.
     * Leave ODR = 0 (power-down / one-shot mode). */
    err = wr(out, REG_CTRL_REG2, CTRL2_SWRESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02x: reset failed: %s", addr, esp_err_to_name(err));
        goto fail;
    }
    for (int i = 0; i < 20; i++) {
        uint8_t ctrl2 = 0;
        vTaskDelay(pdMS_TO_TICKS(5));
        if (rd(out, REG_CTRL_REG2, &ctrl2, 1) == ESP_OK && !(ctrl2 & CTRL2_SWRESET)) {
            break;
        }
    }
    err = wr(out, REG_CTRL_REG2, CTRL2_IF_ADD_INC);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02x: if_add_inc failed: %s", addr, esp_err_to_name(err));
        goto fail;
    }
    err = wr(out, REG_CTRL_REG1, CTRL1_BDU);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x%02x: ctrl1 failed: %s", addr, esp_err_to_name(err));
        goto fail;
    }
    ESP_LOGI(TAG, "0x%02x: LPS2x online", addr);
    return ESP_OK;

fail:
    if (out->dev) {
        esp_err_t rm_err = i2c_master_bus_rm_device(out->dev);
        if (rm_err != ESP_OK) {
            ESP_LOGW(TAG, "0x%02x: failed to release I2C handle: %s", addr, esp_err_to_name(rm_err));
        }
        out->dev = NULL;
    }
    return err;
}

esp_err_t lps2x_deinit(lps2x_t *s)
{
    if (!s || !s->dev) {
        return ESP_OK;
    }

    uint8_t addr = s->addr;
    esp_err_t err = i2c_master_bus_rm_device(s->dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "0x%02x: failed to release I2C handle: %s", addr, esp_err_to_name(err));
    }
    s->dev = NULL;
    s->addr = 0;
    return err;
}

esp_err_t lps2x_read(lps2x_t *s, float *hpa, float *temp_c)
{
    /* kick a one-shot conversion */
    ESP_RETURN_ON_ERROR(wr(s, REG_CTRL_REG2, CTRL2_IF_ADD_INC | CTRL2_ONE_SHOT), TAG, "trigger");

    /* poll for pressure-data-available (max ~200 ms) */
    uint8_t status = 0;
    for (int i = 0; i < 100; i++) {
        if (rd(s, REG_STATUS, &status, 1) == ESP_OK && (status & STATUS_P_DA)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (!(status & STATUS_P_DA)) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t p[3] = {0};
    ESP_RETURN_ON_ERROR(rd(s, REG_PRESS_XL, p, 3), TAG, "press");
    int32_t raw = (int32_t)((uint32_t)p[2] << 16 | (uint32_t)p[1] << 8 | p[0]);
    if (raw & 0x00800000) {            /* sign-extend 24-bit */
        raw |= 0xFF000000;
    }
    *hpa = (float)raw / 4096.0f;       /* LSB = 1/4096 hPa */

    if (temp_c) {
        uint8_t t[2] = {0};
        if (rd(s, REG_TEMP_L, t, 2) == ESP_OK) {
            int16_t traw = (int16_t)((uint16_t)t[1] << 8 | t[0]);
            *temp_c = (float)traw / 100.0f;   /* LSB = 0.01 C */
        }
    }
    return ESP_OK;
}
