/*
 * lps2x.h  -  Minimal driver for ST LPS22HB / LPS33HW / LPS35HW barometric
 * and pressure sensors over I2C (Adafruit LPS22 + LPS35 breakouts).
 *
 * Uses the ESP-IDF 5.x i2c_master driver. Both parts share the same register
 * map, so one driver serves both. Reads are done in one-shot mode to keep the
 * sensors quiet between samples.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
} lps2x_t;

/* Create the shared I2C master bus (call once). */
esp_err_t lps2x_bus_init(i2c_master_bus_handle_t *out_bus);

/* Attach one sensor at the given 7-bit address and verify WHO_AM_I. */
esp_err_t lps2x_init(i2c_master_bus_handle_t bus, uint8_t addr, lps2x_t *out);

/* Trigger a one-shot conversion and return pressure in hPa (and temp in C if
 * temp_c != NULL). Returns ESP_OK on a valid reading. */
esp_err_t lps2x_read(lps2x_t *s, float *hpa, float *temp_c);
