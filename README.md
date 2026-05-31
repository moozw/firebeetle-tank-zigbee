# FireBeetle 2 ESP32-C5 — Zigbee tank-level fill controller

Mains-powered Zigbee **router** that reads water depth from a submerged
Adafruit **LPS35** (referenced against an Adafruit **LPS22** barometer) and
drives a fill pump via an Adafruit **Power Relay FeatherWing**. The pump turns
**ON at a low level** and **OFF at a full level**, both set live over Zigbee.

## How the level is measured

The LPS35 reads absolute pressure at the tank bottom (water head + atmosphere).
The LPS22 reads atmosphere only. Subtracting gives the water column:

```
depth_m = (P_tank − P_baro) [Pa] / (ρ · g)
ρ = density (kg/m³, default 997 fresh water)   g = 9.80665 m/s²
```

Reported as **depth (cm)** and **level (%)** of `tank_height`.

## Wiring (FireBeetle 2 ESP32-C5, SKU DFR1222 — 3.3 V logic)

| Signal | Board pin | GPIO |
|--------|-----------|------|
| I²C SDA (both sensors) | SDA | 9 |
| I²C SCL (both sensors) | SCL | 10 |
| Relay FeatherWing control | D13 | 15 *(also onboard LED = pump indicator)* |
| Override button | onboard **BOOT** | 28 *(no wiring needed)* |

I²C addresses (7-bit):
- **Barometric reference** → **0x5C**
- **Tank/submerged pressure sensor** → **0x5D**

> Confirm with an I²C scan. Both Adafruit boards can power up on the same
> address, so one board must have its address changed or they collide. If your
> scan shows different values, edit `LPS_*_ADDR` in `main/app_config.h`.

## Control logic

```
mode = AUTO:
    depth ≤ level_low   → pump ON   (start filling)
    depth ≥ level_full  → pump OFF  (tank full)
    in between          → hold (hysteresis = the gap between low and full)
mode = FORCE_ON / FORCE_OFF → manual override
fault (5 bad sensor reads) or boot → pump OFF (failsafe)
anti-short-cycle: min 30 s rest before the pump can restart (AUTO only)
```

## Onboard BOOT button (GPIO28) — local override

- **Short press** — cycle mode **AUTO → FORCE ON → FORCE OFF → AUTO**. The new
  mode is saved to NVS and pushed to Zigbee, so the hub's `mode` stays in sync.
  FORCE modes act immediately (they bypass the anti-short-cycle).
- **Long press (≥ 5 s)** — Zigbee factory reset: the device leaves the network
  and reboots ready to re-commission. No re-flash needed.

(The button is also the firmware-download strapping pin, but that only matters
while resetting the board — runtime use is safe.)

## Build & flash (do this **before** plugging the device into mains)

Requires **ESP-IDF v5.5 or newer** (the ESP32-C5 is not in older releases).

```powershell
# from an ESP-IDF terminal
idf.py set-target esp32c5
idf.py build
idf.py -p COMx flash monitor
```

`idf.py reconfigure`/`build` pulls the `esp-zigbee-lib` + `esp-zboss-lib`
managed components automatically (see `main/idf_component.yml`).

## Add to Zigbee2MQTT

1. Copy `zigbee2mqtt/firebeetle_tank.js` to your Z2M
   `data/external_converters/` folder and restart Z2M.
2. Enable "permit join", power the device — it advertises as **DIY / ESP32C5**.
3. In the device page you'll get: `state` (pump), `depth`, `level`, `fault`,
   `baro_pressure`, `tank_pressure`, and settable `level_low`, `level_full`,
   `tank_height`, `density`, `mode`.

To re-commission later, erase and re-flash, or send it through a factory reset
(clear via `idf.py erase-flash`).

## Tunable defaults — `main/app_config.h`

All physical constants live there: pins, I²C addresses, density, sensor mount
offset, default setpoints, sample period, fault limit and pump anti-short-cycle.
Setpoints/height/density/mode are also writable live from Z2M and persist in NVS.

The current checked-in defaults are conservative bench defaults. In particular,
`BENCH_REQUIRE_EQUAL_PRESSURE` keeps AUTO pump control faulted unless both
sensors read close together in air. Set it to `0` before using the relay to
drive an actual pump with one sensor submerged.

## Zigbee OTA

The firmware includes a Zigbee OTA client. For each OTA release:

```powershell
idf.py build
python tools/make_ota.py build/firebeetle_tank_zigbee.bin tank2_v1.0.6.ota --version 0x01000006
```

The `--version` value must match `OTA_FW_VERSION` in `main/app_config.h` and be
higher than the version already running on the device. Put the generated `.ota`
file on the Zigbee2MQTT host and add the printed metadata to the Z2M OTA
override index.

## Files

```
CMakeLists.txt              project
sdkconfig.defaults          Zigbee router + 802.15.4 + partition table
partitions.csv              includes zb_storage / zb_fct for the Zigbee stack
main/app_config.h           ← all the knobs
main/lps2x.[ch]             LPS22/LPS35 I²C driver (one-shot reads)
main/main.c                 control loop + Zigbee model
zigbee2mqtt/firebeetle_tank.js   Z2M external converter
```
