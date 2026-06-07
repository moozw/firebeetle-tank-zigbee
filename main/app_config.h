/*
 * app_config.h  -  All user-tunable constants in one place.
 *
 * Anything that depends on YOUR physical install lives here. The Zigbee
 * setpoints (low/full level) are NOT here - those are set live from
 * Zigbee2MQTT. These are the build-time defaults and the things that
 * can only be known by measuring your tank.
 */
#pragma once

/* ---------- Pin mapping (FireBeetle 2 ESP32-C5, SKU DFR1222) ---------- */
#define I2C_PORT_NUM        0
#define I2C_SDA_GPIO        9      /* board SDA  */
#define I2C_SCL_GPIO        10     /* board SCL  */
#define I2C_FREQ_HZ         10000  /* very slow bus for sensor contact diagnostics  */

#define RELAY_GPIO          15     /* D13 on the FireBeetle = onboard LED too  */
#define RELAY_ACTIVE_HIGH   1      /* Adafruit Power Relay FeatherWing: HIGH = energised */

#define BUTTON_GPIO         28     /* onboard BOOT button, active-low (idle HIGH) */
#define BUTTON_DEBOUNCE_MS  40
#define BUTTON_LONGPRESS_MS 5000   /* >= this = Zigbee factory reset            */

/* ---------- I2C addresses ---------- */
/* Both Adafruit LPS2x boards power up on the same address. Current swapped
 * bench wiring uses 0x5C as the barometric reference and 0x5D as tank. */
#define LPS_BARO_ADDR       0x5C   /* barometric reference                     */
#define LPS_TANK_ADDR       0x5D   /* submerged tank sensor                    */

/* ---------- Hydrostatic depth calculation ---------- */
/* depth_m = (P_tank - P_baro) / (rho * g)                                    */
#define GRAVITY_MS2             9.80665f
#define DEFAULT_WATER_DENSITY   997    /* kg/m^3. Fresh ~997, seawater ~1025. Z-writable */
#define SENSOR_OFFSET_CM        0      /* height of the sensor above the tank floor */

/* ---------- Default level setpoints (cm of water above sensor) ---------- */
/* These are just power-on defaults; change them live from Zigbee2MQTT.       */
#define DEFAULT_LEVEL_LOW_CM    20     /* relay (pump) turns ON  at/below this  */
#define DEFAULT_LEVEL_FULL_CM   80     /* relay (pump) turns OFF at/above this  */
#define DEFAULT_TANK_HEIGHT_CM  250    /* used to compute level % and sanity checks */

/* ---------- Timing / safety ---------- */
#define SAMPLE_PERIOD_MS        10000  /* how often we read the sensors         */
/* DIAGNOSTIC: when set, in AUTO mode with no full water level the relay/LED
 * mirrors "is any sensor reading?" - a Zigbee-visible sensor-alive indicator.
 * Set to 0 for normal failsafe-off behaviour once bench testing is done. */
#define DIAG_RELAY_SENSOR_INDICATOR 0
/* delay app->stack ZCL traffic after join so the coordinator interview finishes
 * first; pushing reports mid-interview asserts the stack and reboots the device */
#define TELEMETRY_START_DELAY_US (30LL * 1000000)
#define SENSOR_FAULT_LIMIT      3      /* consecutive bad reads -> failsafe OFF */
#define SENSOR_AVG_SAMPLES      3      /* one-shot readings per displayed sample */
#define SENSOR_AVG_DELAY_MS     250    /* rest between one-shot conversions      */
#define MAX_DEPTH_OVER_TANK_CM  20     /* depth above tank height => fault       */
#define BENCH_REQUIRE_EQUAL_PRESSURE 0  /* setup calibration now handles offsets  */
#define BENCH_MAX_DELTA_HPA     10.0f  /* both sensors in air should be close    */
#define PUMP_MIN_OFF_MS         30000  /* anti-short-cycle: min rest before re-ON */
#define PUMP_MIN_ON_MS          0      /* optional min run time (0 = disabled)  */
#define LOG_SENSOR_READINGS     1      /* diagnostic serial log of pressure/depth */

/* ---------- Local setup portal ---------- */
/* First-pass installer UI: phone connects to this AP and opens http://192.168.4.1.
 * It stores calibration/config in the same NVS blob used by Zigbee. */
#define SETUP_PORTAL_ENABLE     1
#define SETUP_AP_SSID_PREFIX    "TankSetup"
#define SETUP_AP_PASSWORD       "tanksetup"
#define SETUP_AP_CHANNEL        6
#define SETUP_AP_MAX_CONN       2

/* ---------- Zigbee ---------- */
#define ZB_ENDPOINT             10
#define ZB_PRIMARY_CHANNEL_MASK (1l << 15)   /* try ch 15 first; full mask also set */
#define ZB_MANUF_CODE           0x1224       /* manufacturer code for custom attrs */
#define ZB_CUSTOM_CLUSTER_ID    0xFC11       /* our private telemetry/config cluster */

/* Custom cluster attribute IDs */
#define ATTR_DEPTH_CM           0x0000  /* s16, read-only, reported   */
#define ATTR_LEVEL_PCT          0x0001  /* u8,  read-only, reported   */
#define ATTR_FAULT              0x0002  /* u8,  0=ok 1=sensor fault   */
#define ATTR_BARO_PRESSURE_HPA  0x0003  /* s16, read-only, reported   */
#define ATTR_TANK_PRESSURE_HPA  0x0004  /* s16, read-only, reported   */
#define ATTR_SET_LOW_CM         0x0010  /* s16, read/write            */
#define ATTR_SET_FULL_CM        0x0011  /* s16, read/write            */
#define ATTR_TANK_HEIGHT_CM     0x0012  /* s16, read/write            */
#define ATTR_DENSITY            0x0013  /* u16, read/write (kg/m^3)   */
#define ATTR_MODE               0x0014  /* u8: 0=auto 1=force-on 2=force-off */

/* control modes */
#define MODE_AUTO   0
#define MODE_ON     1
#define MODE_OFF    2

/* connectivity preference saved by setup UI; current firmware still starts
 * Zigbee and setup AP, but this records the intended operating mode. */
#define CONN_STANDALONE 0
#define CONN_ZIGBEE     1
#define CONN_WIFI       2
#define CONN_BOTH       3

/* ---------- OTA (wireless firmware update over Zigbee) ---------- */
/* Bump OTA_FW_VERSION for every release you want to push over the air, then
 * rebuild and run tools/make_ota.py with the same version. Z2M offers the
 * update when the packaged .ota version is higher than what the device runs. */
#define OTA_FW_VERSION      0x0100000B   /* 1.0.0.11 - setup portal calibration */
#define OTA_MANUF_CODE      0x1224       /* OTA manufacturer code */
#define OTA_IMAGE_TYPE      0x1011       /* OTA image type id     */
#define OTA_HW_VERSION      0x0101
#define OTA_MAX_DATA_SIZE   223          /* max OTA block payload bytes */
#define OTA_QUERY_INTERVAL_MIN 5         /* client retry interval        */
#define OTA_VERSION_ZCL_STRING "\x08""1.0.0.11"
