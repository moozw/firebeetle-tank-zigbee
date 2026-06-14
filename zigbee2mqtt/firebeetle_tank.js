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
//   0x06 external_temperature(C x100,s16 RO)  0x07 water_temperature(C x100,s16 RO)
//   0x10 level_low(cm,s16) 0x11 level_full(cm,s16) 0x12 tank_height(cm,s16)
//   0x13 density(kg/m3,u16) 0x14 mode(u8: 0 auto/1 force_on/2 force_off)
//   0x15 operating_level(cm,s16)
//   0x16 pump_lockout(u8) 0x19 lockout_active(u8 RO) 0x1A time_valid(u8 RO)

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
const TEMP_UNAVAILABLE = -32768;

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
        const externalTemp = firstDefined(d, ['6', 'attr6']);
        const waterTemp = firstDefined(d, ['7', 'attr7']);
        if (depth     !== undefined) r.depth       = depth;
        if (level     !== undefined) r.level       = level;
        if (fault     !== undefined) r.fault       = fault ? 'ON' : 'OFF';
        if (baroPressure !== undefined) r.baro_pressure = baroPressure;
        if (tankPressure !== undefined) r.tank_pressure = tankPressure;
        if (lowAlert !== undefined) r.low_alert = lowAlert ? 'ON' : 'OFF';
        if (externalTemp !== undefined && externalTemp !== TEMP_UNAVAILABLE) r.external_temperature = externalTemp / 100;
        if (waterTemp !== undefined && waterTemp !== TEMP_UNAVAILABLE) r.water_temperature = waterTemp / 100;
        // config setpoints - names mirror the WiFi setup portal labels
        if (d['16'] !== undefined) r.low_alert_cm   = d['16'];   // 0x10 "Low alert cm"
        if (d['17'] !== undefined) r.full_cm        = d['17'];   // 0x11 "Full level cm"
        if (d['18'] !== undefined) r.tank_height_cm = d['18'];   // 0x12 "Tank height cm"
        if (d['19'] !== undefined) r.density        = d['19'];   // 0x13 "Density"
        if (d['20'] !== undefined) r.mode           = MODE_MAP[d['20']]; // 0x14 "Mode"
        if (d['21'] !== undefined) r.operating_cm   = d['21'];   // 0x15 "Operating level cm"
        if (d['22'] !== undefined) r.pump_lockout   = d['22'] ? 'ON' : 'OFF'; // 0x16
        if (d['25'] !== undefined) r.lockout_active = d['25'] ? 'ON' : 'OFF'; // 0x19
        if (d['26'] !== undefined) r.time_valid     = d['26'] ? 'ON' : 'OFF'; // 0x1A
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

// depth/level live on standard genAnalogInput endpoints (11/12) so the stack
// auto-reports them (the custom 0xFC11 cluster does not).
const fzAnalog = {
    cluster: 'genAnalogInput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const v = msg.data['presentValue'];
        if (v === undefined) return {};
        if (msg.endpoint.ID === 11) return {depth: v};
        if (msg.endpoint.ID === 12) return {level: v};
        return {};
    },
};

// water/reference temperature on standard temperature-measurement endpoints (13/14)
const fzTempEp = {
    cluster: 'msTemperatureMeasurement',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const v = msg.data['measuredValue'];
        if (v === undefined || v === -32768) return {};
        const c = v / 100;
        if (msg.endpoint.ID === 13) return {water_temperature: c};
        if (msg.endpoint.ID === 14) return {external_temperature: c};
        return {};
    },
};

// settable config attributes
const tzTank = {
    key: ['low_alert_cm', 'operating_cm', 'full_cm', 'tank_height_cm', 'density', 'mode', 'pump_lockout'],
    convertSet: async (entity, key, value, meta) => {
        const W = {
            low_alert_cm:   [0x10, S16], full_cm:       [0x11, S16],
            tank_height_cm: [0x12, S16], density:       [0x13, U16], mode: [0x14, U8],
            operating_cm:   [0x15, S16], pump_lockout:  [0x16, U8],
        };
        const [attr, type] = W[key];
        const raw = key === 'mode' ? MODE_REV[value] : key === 'pump_lockout' ? (value === 'ON' || value === true ? 1 : 0) : value;
        // NB: firmware registers 0xFC11 attrs as plain (non-manufacturer-specific),
        // so reads/writes must NOT carry a manufacturerCode or the device replies
        // UNSUPPORTED_ATTRIBUTE. (MANUFACTURER_CODE kept for reference only.)
        await entity.write(CLUSTER, {[attr]: {value: raw, type}});
        return {state: {[key]: value}};
    },
    convertGet: async (entity, key, meta) => {
        const R = {low_alert_cm: 0x10, full_cm: 0x11, tank_height_cm: 0x12, density: 0x13, mode: 0x14, operating_cm: 0x15, pump_lockout: 0x16};
        await entity.read(CLUSTER, [R[key]]);
    },
};

// read-only telemetry (refresh button reads it on demand)
const tzTankRead = {
    key: ['depth', 'level', 'fault', 'low_alert', 'baro_pressure', 'tank_pressure', 'external_temperature', 'water_temperature', 'lockout_active', 'time_valid'],
    convertGet: async (entity, key, meta) => {
        if (key === 'baro_pressure') {
            await entity.read(PRESSURE_CLUSTER, [0x0000]);
            return;
        }
        if (key === 'tank_pressure') {
            await entity.read(PRESSURE_CLUSTER, [0x0010]);
            return;
        }
        const R = {depth: 0x00, level: 0x01, fault: 0x02, baro_pressure: 0x03, tank_pressure: 0x04, low_alert: 0x05, external_temperature: 0x06, water_temperature: 0x07, lockout_active: 0x19, time_valid: 0x1A};
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
        fromZigbee: [fz.on_off, fzTank, fzPressure, fzAnalog, fzTempEp],
        toZigbee: [tz.on_off, tzTank, tzTankRead],
        configure: async (device, coordinatorEndpoint) => {
            const ep = device.getEndpoint(10);
            // relay state reporting (standard cluster)
            try { await ep.bind('genOnOff', coordinatorEndpoint); await reporting.onOff(ep); } catch (e) {}
            try {
                await ep.bind(PRESSURE_CLUSTER, coordinatorEndpoint);
                await ep.configureReporting(PRESSURE_CLUSTER, [
                    {attribute: {ID: 0x0000, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                    {attribute: {ID: 0x0010, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                ]);
            } catch (e) {}
            // custom telemetry auto-reporting (best effort)
            try {
                await ep.bind(CLUSTER, coordinatorEndpoint);
                await ep.configureReporting(CLUSTER, [
                    {attribute: {ID: 0x00, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                    {attribute: {ID: 0x01, type: U8},  minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                    {attribute: {ID: 0x02, type: U8},  minimumReportInterval: 0, maximumReportInterval: 120, reportableChange: 0},
                    {attribute: {ID: 0x03, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                    {attribute: {ID: 0x04, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1},
                    {attribute: {ID: 0x05, type: U8},  minimumReportInterval: 0, maximumReportInterval: 120, reportableChange: 0},
                    {attribute: {ID: 0x06, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 25},
                    {attribute: {ID: 0x07, type: S16}, minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 25},
                    {attribute: {ID: 0x19, type: U8},  minimumReportInterval: 0, maximumReportInterval: 120, reportableChange: 0},
                    {attribute: {ID: 0x1A, type: U8},  minimumReportInterval: 0, maximumReportInterval: 120, reportableChange: 0},
                ]);
            } catch (e) {}
            // standard reportable sensor endpoints - these actually auto-report
            for (const epId of [11, 12]) {
                try {
                    const se = device.getEndpoint(epId);
                    await se.bind('genAnalogInput', coordinatorEndpoint);
                    await se.configureReporting('genAnalogInput', [{attribute: 'presentValue', minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1}]);
                } catch (e) {}
            }
            for (const epId of [13, 14]) {
                try {
                    const se = device.getEndpoint(epId);
                    await se.bind('msTemperatureMeasurement', coordinatorEndpoint);
                    await se.configureReporting('msTemperatureMeasurement', [{attribute: 'measuredValue', minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 25}]);
                } catch (e) {}
            }
        },
        exposes: [
            e.switch(),
            e.numeric('depth', ea.STATE_GET).withUnit('cm').withDescription('Water depth above sensor'),
            e.numeric('level', ea.STATE_GET).withUnit('%').withDescription('Tank level'),
            e.binary('fault', ea.STATE_GET, 'ON', 'OFF').withDescription('Sensor fault (pump forced off)'),
            e.binary('low_alert', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Low alert active (depth at/below the Low alert level)'),
            e.binary('pump_lockout', ea.ALL, 'ON', 'OFF')
                .withDescription('When ON, AUTO mode will not run the pump'),
            e.binary('lockout_active', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Pump lockout is currently blocking AUTO pump operation'),
            e.binary('time_valid', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Local clock is valid for WiFi scheduled lockout'),
            e.numeric('baro_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Barometric reference pressure'),
            e.numeric('tank_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Tank sensor absolute pressure'),
            e.numeric('external_temperature', ea.STATE_GET).withUnit('C')
                .withDescription('External/reference sensor temperature'),
            e.numeric('water_temperature', ea.STATE_GET).withUnit('C')
                .withDescription('Water/tank sensor temperature'),
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
