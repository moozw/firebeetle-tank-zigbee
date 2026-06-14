// Zigbee2MQTT external converter — FireBeetle 2 ESP32-C5 tank controller.
// Classic fromZigbee/toZigbee format for maximum Z2M-version compatibility.
//
// Install:  copy to <z2m-data>/external_converters/firebeetle_tank.js, restart Z2M,
//           then "Reconfigure" the device once (sets up reporting).
//
// Custom cluster 0xFC11 (51217) attribute IDs:
//   0x00 depth(cm,s16 RO)  0x01 level(%,u8 RO)  0x02 fault(u8 RO)
//   0x03 baro_pressure(hPa,s16 RO)  0x04 tank_pressure(hPa,s16 RO)
//   0x05 low_alert(u8 RO)
//   0x10 level_low(cm,s16) 0x11 level_full(cm,s16) 0x12 tank_height(cm,s16)
//   0x13 density(kg/m3,u16) 0x14 mode(u8: 0 auto/1 force_on/2 force_off)
//   0x15 operating_level(cm,s16)

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const ea = exposes.access;

const CLUSTER = 0xFC11;
const PRESSURE_CLUSTER = 0x0403;
const MANUFACTURER_CODE = 0x1224;
const MODE_MAP = {0: 'auto', 1: 'force_on', 2: 'force_off'};
const MODE_REV = {auto: 0, force_on: 1, force_off: 2};
const S16 = 0x29, U16 = 0x21, U8 = 0x20;

const firstDefined = (data, keys) => {
    for (const key of keys) {
        if (data[key] !== undefined) return data[key];
    }
    return undefined;
};

const fzTank = {
    cluster: CLUSTER,
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const d = msg.data;
        const r = {};
        const depth = firstDefined(d, ['0', 'attr0']);
        const level = firstDefined(d, ['1', 'attr1']);
        const fault = firstDefined(d, ['2', 'attr2']);
        const baroPressure = firstDefined(d, ['3', 'attr3']);
        const tankPressure = firstDefined(d, ['4', 'attr4']);
        const lowAlert = firstDefined(d, ['5', 'attr5']);
        if (depth     !== undefined) r.depth       = depth;
        if (level     !== undefined) r.level       = level;
        if (fault     !== undefined) r.fault       = fault ? 'ON' : 'OFF';
        if (baroPressure !== undefined) r.baro_pressure = baroPressure;
        if (tankPressure !== undefined) r.tank_pressure = tankPressure;
        if (lowAlert !== undefined) r.low_alert = lowAlert ? 'ON' : 'OFF';
        // config setpoints - names mirror the WiFi setup portal labels
        if (d['16'] !== undefined) r.low_alert_cm   = d['16'];   // 0x10 "Low alert cm"
        if (d['17'] !== undefined) r.full_cm        = d['17'];   // 0x11 "Full level cm"
        if (d['18'] !== undefined) r.tank_height_cm = d['18'];   // 0x12 "Tank height cm"
        if (d['19'] !== undefined) r.density        = d['19'];   // 0x13 "Density"
        if (d['20'] !== undefined) r.mode           = MODE_MAP[d['20']]; // 0x14 "Mode"
        if (d['21'] !== undefined) r.operating_cm   = d['21'];   // 0x15 "Operating level cm"
        return r;
    },
};

const fzPressure = {
    cluster: 'msPressureMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const d = msg.data;
        const r = {};
        if (d.measuredValue !== undefined) r.baro_pressure = d.measuredValue;
        if (d.scaledValue !== undefined) r.tank_pressure = d.scaledValue;
        return r;
    },
};

// settable config attributes
const tzTank = {
    key: ['low_alert_cm', 'operating_cm', 'full_cm', 'tank_height_cm', 'density', 'mode'],
    convertSet: async (entity, key, value, meta) => {
        const W = {
            low_alert_cm:   [0x10, S16], full_cm:       [0x11, S16],
            tank_height_cm: [0x12, S16], density:       [0x13, U16], mode: [0x14, U8],
            operating_cm:   [0x15, S16],
        };
        const [attr, type] = W[key];
        const raw = key === 'mode' ? MODE_REV[value] : value;
        // NB: firmware registers 0xFC11 attrs as plain (non-manufacturer-specific),
        // so reads/writes must NOT carry a manufacturerCode or the device replies
        // UNSUPPORTED_ATTRIBUTE. (MANUFACTURER_CODE kept for reference only.)
        await entity.write(CLUSTER, {[attr]: {value: raw, type}});
        return {state: {[key]: value}};
    },
    convertGet: async (entity, key, meta) => {
        const R = {low_alert_cm: 0x10, full_cm: 0x11, tank_height_cm: 0x12, density: 0x13, mode: 0x14, operating_cm: 0x15};
        await entity.read(CLUSTER, [R[key]]);
    },
};

// read-only telemetry (refresh button reads it on demand)
const tzTankRead = {
    key: ['depth', 'level', 'fault', 'low_alert', 'baro_pressure', 'tank_pressure'],
    convertGet: async (entity, key, meta) => {
        if (key === 'baro_pressure') {
            await entity.read(PRESSURE_CLUSTER, [0x0000]);
            return;
        }
        if (key === 'tank_pressure') {
            await entity.read(PRESSURE_CLUSTER, [0x0010]);
            return;
        }
        const R = {depth: 0x00, level: 0x01, fault: 0x02, baro_pressure: 0x03, tank_pressure: 0x04, low_alert: 0x05};
        await entity.read(CLUSTER, [R[key]]);
    },
};

module.exports = [
    {
        zigbeeModel: ['ESP32C5'],
        model: 'FB2-C5-TANK',
        vendor: 'DIY',
        description: 'FireBeetle 2 ESP32-C5 underwater tank level / fill relay',
        ota: true,                 // enable wireless firmware updates (served from the OTA override index)
        fromZigbee: [fz.on_off, fzTank, fzPressure],
        toZigbee: [tz.on_off, tzTank, tzTankRead],
        configure: async (device, coordinatorEndpoint) => {
            const ep = device.getEndpoint(10);
            // relay state reporting (standard cluster)
            try { await ep.bind('genOnOff', coordinatorEndpoint); await reporting.onOff(ep); } catch (e) {}
            try {
                await ep.bind(PRESSURE_CLUSTER, coordinatorEndpoint);
                await ep.configureReporting(PRESSURE_CLUSTER, [
                    {attribute: {ID: 0x0000, type: S16}, minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x0010, type: S16}, minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                ]);
            } catch (e) {}
            // custom telemetry auto-reporting (best effort)
            try {
                await ep.bind(CLUSTER, coordinatorEndpoint);
                await ep.configureReporting(CLUSTER, [
                    {attribute: {ID: 0x00, type: S16}, minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x01, type: U8},  minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x02, type: U8},  minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
                    {attribute: {ID: 0x03, type: S16}, minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x04, type: S16}, minimumReportInterval: 0, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x05, type: U8},  minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
                ]);
            } catch (e) {}
        },
        exposes: [
            e.switch(),
            e.numeric('depth', ea.STATE_GET).withUnit('cm').withDescription('Water depth above sensor'),
            e.numeric('level', ea.STATE_GET).withUnit('%').withDescription('Tank level'),
            e.binary('fault', ea.STATE_GET, 'ON', 'OFF').withDescription('Sensor fault (pump forced off)'),
            e.binary('low_alert', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Low alert active (depth at/below the Low alert level)'),
            e.numeric('baro_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Barometric reference pressure'),
            e.numeric('tank_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Tank sensor absolute pressure'),
            e.numeric('low_alert_cm', ea.ALL).withUnit('cm').withValueMin(0).withValueMax(300)
                .withDescription('Low alert level (WiFi setup: "Low alert cm")'),
            e.numeric('operating_cm', ea.ALL).withUnit('cm').withValueMin(0).withValueMax(300)
                .withDescription('Operating level - pump ON at/below (WiFi setup: "Operating level cm")'),
            e.numeric('full_cm', ea.ALL).withUnit('cm').withValueMin(0).withValueMax(300)
                .withDescription('Full level - pump OFF at/above (WiFi setup: "Full level cm")'),
            e.numeric('tank_height_cm', ea.ALL).withUnit('cm').withValueMin(1).withValueMax(500)
                .withDescription('Tank height (WiFi setup: "Tank height cm")'),
            e.numeric('density', ea.ALL).withUnit('kg/m3').withValueMin(900).withValueMax(1100)
                .withDescription('Density kg/m3 (fresh ~997, sea ~1025)'),
            e.enum('mode', ea.ALL, ['auto', 'force_on', 'force_off'])
                .withDescription('Mode (auto / force on / force off)'),
        ],
    },
];
