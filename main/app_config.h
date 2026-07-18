/*
 * app_config.h  -  All user-tunable constants in one place.
 *
 * Anything that depends on YOUR physical install lives here. The Zigbee
 * setpoints (low alert / operating / full level) are NOT here - those are set live from
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
#define BUTTON_CLICK_MAX_MS 1000   /* longer holds cancel the click sequence    */
#define BUTTON_MULTI_CLICK_MS 700  /* wait after release for another click      */

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
#define DEFAULT_LEVEL_LOW_CM    20     /* alert only at/below this depth       */
#define DEFAULT_OPERATING_CM    30     /* relay (pump) turns ON at/below this  */
#define DEFAULT_LEVEL_FULL_CM   80     /* relay (pump) turns OFF at/above this */
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
#define SENSOR_MIN_VALID_HPA    300.0f /* lower values are missed/invalid reads   */
#define SENSOR_MAX_DROP_CM      50     /* larger one-cycle drops are bad reads    */
#define MAX_DEPTH_OVER_TANK_CM  20     /* depth above tank height => fault       */
#define FULL_INHIBIT_CLEAR_READINGS 5  /* fresh <100% reads before pump can run  */
#define BENCH_REQUIRE_EQUAL_PRESSURE 0  /* setup calibration now handles offsets  */
#define BENCH_MAX_DELTA_HPA     10.0f  /* both sensors in air should be close    */
#define PUMP_MIN_OFF_MS         30000  /* anti-short-cycle: min rest before re-ON */
#define PUMP_MIN_ON_MS          0      /* optional min run time (0 = disabled)  */
#define DEFAULT_LOCKOUT_START_MIN 1320 /* 22:00 local time */
#define DEFAULT_LOCKOUT_END_MIN   300  /* 05:00 local time */
#define LOCAL_TIMEZONE_POSIX    "CAT-2" /* Africa/Harare UTC+2, POSIX sign */
#define LOCAL_TIMEZONE_NAME     "Africa/Harare"
#define SNTP_SYNC_INTERVAL_MS   (30UL * 60UL * 1000UL)
#define LOG_SENSOR_READINGS     1      /* diagnostic serial log of pressure/depth */

/* ---------- Local setup portal ---------- */
/* First-pass installer UI: phone connects to this AP and opens http://192.168.4.1.
 * It stores calibration/config in the same NVS blob used by Zigbee. */
#define SETUP_PORTAL_ENABLE     1
#define SETUP_AP_SSID_PREFIX    "TankSetup"
#define SETUP_AP_PASSWORD       "tanksetup"
#define SETUP_AP_CHANNEL        6
#define SETUP_AP_MAX_CONN       2

/* Web config-page login. Enforced ONLY when the device is on local WiFi (STA);
 * the temporary setup AP stays open (you physically join it, and it shuts off
 * after setup). Ships with this default shown on the page; the user is forced to
 * change it before any settings can be saved over WiFi. Username is fixed. */
#define SETUP_WEB_USER          "admin"
#define SETUP_WEB_DEFAULT_PASS  "tankadmin"

/* ---------- Local WiFi / MQTT ---------- */
#define WIFI_CONNECT_TIMEOUT_MS 15000
#define WIFI_MAX_RETRY          5
#define DEFAULT_MQTT_HOST       ""
#define DEFAULT_MQTT_TOPIC      "tank/controller"
#define MQTT_PUBLISH_PERIOD_MS  10000

/* ---------- Zigbee ---------- */
#define ZB_ENDPOINT             10
/* Extra endpoints carrying STANDARD reportable clusters so the esp-zigbee stack
 * auto-reports them (the custom 0xFC11 cluster does not auto-report). */
#define ZB_EP_DEPTH             11   /* genAnalogInput  - depth cm   */
#define ZB_EP_LEVEL             12   /* genAnalogInput  - level %    */
#define ZB_EP_WATER_TEMP        13   /* msTemperatureMeasurement     */
#define ZB_EP_AIR_TEMP          14   /* msTemperatureMeasurement     */
#define ZB_EP_FAULT             15   /* genAnalogInput - fault code  */
#define ZB_EP_BARO_PRESSURE     16   /* genAnalogInput - hPa         */
#define ZB_EP_TANK_PRESSURE     17   /* genAnalogInput - hPa         */
#define ZB_EP_LOW_ALERT         18   /* genAnalogInput - boolean     */
#define ZB_EP_RELAY             19   /* genAnalogInput - boolean     */
#define ZB_EP_MODE              20   /* genAnalogInput - mode enum   */
#define ZB_EP_SET_LOW           23   /* genAnalogOutput - cm         */
#define ZB_EP_SET_OPERATING     24   /* genAnalogOutput - cm         */
#define ZB_EP_SET_FULL          25   /* genAnalogOutput - cm         */
#define ZB_EP_SET_TANK_HEIGHT   26   /* genAnalogOutput - cm         */
#define ZB_EP_SET_DENSITY       27   /* genAnalogOutput - kg/m3      */
#define ZB_EP_SET_MODE          28   /* genAnalogOutput - mode enum  */
#define ZB_PRIMARY_CHANNEL_MASK (1l << 15)   /* try ch 15 first; full mask also set */
#define ZB_MANUF_CODE           0x1224       /* manufacturer code for custom attrs */
#define ZB_CUSTOM_CLUSTER_ID    0xFC11       /* our private telemetry/config cluster */

/* Custom cluster attribute IDs */
#define ATTR_DEPTH_CM           0x0000  /* s16, read-only, reported   */
#define ATTR_LEVEL_PCT          0x0001  /* u8,  read-only, reported   */
#define ATTR_FAULT              0x0002  /* u8,  0=ok 1=sensor fault   */
#define ATTR_BARO_PRESSURE_HPA  0x0003  /* s16, read-only, reported   */
#define ATTR_TANK_PRESSURE_HPA  0x0004  /* s16, read-only, reported   */
#define ATTR_LOW_ALERT          0x0005  /* u8,  read-only, reported   */
#define ATTR_AIR_TEMP_CX100     0x0006 /* s16, read-only, reported, degC x100 */
#define ATTR_WATER_TEMP_CX100   0x0007  /* s16, read-only, reported, degC x100 */
#define ATTR_RELAY_STATE        0x0008  /* u8,  read-only controller output     */
#define ATTR_SET_LOW_CM         0x0010  /* s16, read/write            */
#define ATTR_SET_FULL_CM        0x0011  /* s16, read/write            */
#define ATTR_TANK_HEIGHT_CM     0x0012  /* s16, read/write            */
#define ATTR_DENSITY            0x0013  /* u16, read/write (kg/m^3)   */
#define ATTR_MODE               0x0014  /* u8: 0=auto 1=force-on 2=force-off */
#define ATTR_OPERATING_CM       0x0015  /* s16, read/write            */
#define ATTR_LOCKOUT_ENABLED    0x0016  /* u8,  read/write            */
#define ATTR_LOCKOUT_START_MIN  0x0017  /* u16, read/write minute of day */
#define ATTR_LOCKOUT_END_MIN    0x0018  /* u16, read/write minute of day */
#define ATTR_LOCKOUT_ACTIVE     0x0019  /* u8,  read-only, reported   */
#define ATTR_TIME_VALID         0x001A  /* u8,  read-only, reported   */

/* control modes */
#define MODE_AUTO   0
#define MODE_ON     1
#define MODE_OFF    2

/* connectivity preference saved by setup UI. A fresh/unset unit starts the
 * setup AP. CONN_AP means standalone/offline after setup; the AP only returns
 * for physical service button access or full reset. */
#define CONN_AP         0
#define CONN_ZIGBEE     1
#define CONN_WIFI       2
#define CONN_UNSET      255

/* ---------- OTA (wireless firmware update over Zigbee) ---------- */
/* Bump OTA_FW_VERSION for every release you want to push over the air, then
 * rebuild and run tools/make_ota.py with the same version. Z2M offers the
 * update when the packaged .ota version is higher than what the device runs. */
#define OTA_FW_VERSION      0x0100002C   /* 1.0.0.44 - I2C fault diagnostics */
#define OTA_MANUF_CODE      0x1224       /* OTA manufacturer code */
#define OTA_IMAGE_TYPE      0x1011       /* OTA image type id     */
#define OTA_HW_VERSION      0x0101
#define OTA_MAX_DATA_SIZE   223          /* max OTA block payload bytes */
#define OTA_QUERY_INTERVAL_S 60          /* delay after server discovery */
#define OTA_VERSION_ZCL_STRING "\x08""1.0.0.44"
