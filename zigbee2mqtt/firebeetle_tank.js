// Zigbee2MQTT external converter - FireBeetle 2 ESP32-C5 tank controller.
// Live values use standard Analog Input and Temperature Measurement endpoints.
// Settings use standard writable Analog Output endpoints; the endpoint number
// supplies the tank-specific meaning. The normal On/Off cluster is AUTO lockout.
//
// Install in <z2m-data>/external_converters/, restart Z2M, then re-pair the
// device so the coordinator interviews the complete standard endpoint list.

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const e = exposes.presets;
const ea = exposes.access;

const MODE_MAP = {0: 'auto', 1: 'force_on', 2: 'force_off'};
const MODE_REV = {auto: 0, force_on: 1, force_off: 2};
const FAULT_MAP = {
    0: 'ok', 1: 'sensor_read_failure', 2: 'single_sensor',
    3: 'depth_over_range', 4: 'sensor_mismatch', 5: 'not_calibrated',
};
const INPUT_EP = {
    depth: 11, level: 12, fault: 15, baro_pressure: 16,
    tank_pressure: 17, low_alert: 18, relay: 19, mode: 20,
};
const OUTPUT_EP = {
    low_alert_cm: 23, operating_cm: 24, full_cm: 25,
    tank_height_cm: 26, density: 27, mode: 28,
};

const fzAnalogInput = {
    cluster: 'genAnalogInput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const v = msg.data['presentValue'];
        if (v === undefined) return {};
        const ep = msg.endpoint.ID;
        if (ep === INPUT_EP.depth) return {depth: v};
        if (ep === INPUT_EP.level) return {level: v};
        if (ep === INPUT_EP.fault) return {fault: FAULT_MAP[Math.round(v)] || 'unknown'};
        if (ep === INPUT_EP.baro_pressure) return {baro_pressure: v};
        if (ep === INPUT_EP.tank_pressure) return {tank_pressure: v};
        if (ep === INPUT_EP.low_alert) return {low_alert: v ? 'ON' : 'OFF'};
        if (ep === INPUT_EP.relay) return {relay: v ? 'ON' : 'OFF'};
        if (ep === INPUT_EP.mode) return {mode: MODE_MAP[Math.round(v)] || 'auto'};
        return {};
    },
};

const fzAnalogOutput = {
    cluster: 'genAnalogOutput',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg, publish, options, meta) => {
        const v = msg.data['presentValue'];
        if (v === undefined) return {};
        const key = Object.keys(OUTPUT_EP).find((name) => OUTPUT_EP[name] === msg.endpoint.ID);
        if (!key) return {};
        return {[key]: key === 'mode' ? (MODE_MAP[Math.round(v)] || 'auto') : Math.round(v)};
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
        if (msg.endpoint.ID === 14) return {air_temperature: c};
        return {};
    },
};

const fzLockout = {
    cluster: 'genOnOff',
    type: ['attributeReport', 'readResponse'],
    convert: (model, msg) => msg.data.onOff === undefined ? {} :
        {lockout: msg.data.onOff ? 'ON' : 'OFF'},
};

// settable config attributes
const tzTank = {
    key: Object.keys(OUTPUT_EP),
    convertSet: async (entity, key, value, meta) => {
        const endpoint = meta.device.getEndpoint(OUTPUT_EP[key]);
        const raw = key === 'mode' ? MODE_REV[value] : Number(value);
        await endpoint.write('genAnalogOutput', {presentValue: raw});
        return {state: {[key]: value}};
    },
    convertGet: async (entity, key, meta) => {
        await meta.device.getEndpoint(OUTPUT_EP[key]).read('genAnalogOutput', ['presentValue']);
    },
};

// read-only telemetry (refresh button reads it on demand)
const tzTankRead = {
    key: ['depth', 'level', 'fault', 'low_alert', 'baro_pressure', 'tank_pressure', 'air_temperature', 'water_temperature', 'relay'],
    convertGet: async (entity, key, meta) => {
        if (key === 'air_temperature' || key === 'water_temperature') {
            const ep = key === 'water_temperature' ? 13 : 14;
            await meta.device.getEndpoint(ep).read('msTemperatureMeasurement', ['measuredValue']);
            return;
        }
        await meta.device.getEndpoint(INPUT_EP[key]).read('genAnalogInput', ['presentValue']);
    },
};

const tzLockout = {
    key: ['lockout'],
    convertSet: async (entity, key, value, meta) => {
        const lockout = String(value).toUpperCase();
        if (lockout !== 'ON' && lockout !== 'OFF') {
            throw new Error(`Invalid lockout value: ${value}`);
        }
        await tz.on_off.convertSet(entity, 'state', lockout, meta);
        return {state: {lockout}};
    },
    convertGet: async (entity, key, meta) => {
        await tz.on_off.convertGet(entity, 'state', meta);
    },
};

module.exports = [
    {
        zigbeeModel: ['ESP32C5'],
        model: 'FB2-C5-TANK',
        vendor: 'DIY',
        description: 'FireBeetle 2 ESP32-C5 underwater tank level / fill relay',
        ota: true,                 // enable wireless firmware updates (served from the OTA override index)
        fromZigbee: [fzLockout, fzAnalogInput, fzAnalogOutput, fzTempEp],
        // The standard on/off switch controls AUTO lockout. It does not drive
        // the relay; AUTO/force behavior remains owned by the firmware.
        toZigbee: [tzLockout, tzTank, tzTankRead],
        configure: async (device, coordinatorEndpoint) => {
            const ep = device.getEndpoint(10);
            try {
                await ep.bind('genOnOff', coordinatorEndpoint);
                await reporting.onOff(ep);
            } catch (error) {
                console.warn(`Tank lockout reporting setup failed: ${error.message}`);
            }
            for (const epId of Object.values(INPUT_EP)) {
                try {
                    const se = device.getEndpoint(epId);
                    await se.bind('genAnalogInput', coordinatorEndpoint);
                    await se.configureReporting('genAnalogInput', [{attribute: 'presentValue', minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 1}]);
                    await se.read('genAnalogInput', ['presentValue']);
                } catch (error) {
                    console.warn(`Tank input endpoint ${epId} setup failed: ${error.message}`);
                }
            }
            for (const epId of [13, 14]) {
                try {
                    const se = device.getEndpoint(epId);
                    await se.bind('msTemperatureMeasurement', coordinatorEndpoint);
                    await se.configureReporting('msTemperatureMeasurement', [{attribute: 'measuredValue', minimumReportInterval: 0, maximumReportInterval: 30, reportableChange: 25}]);
                    await se.read('msTemperatureMeasurement', ['measuredValue']);
                } catch (error) {
                    console.warn(`Tank temperature endpoint ${epId} setup failed: ${error.message}`);
                }
            }
            for (const epId of Object.values(OUTPUT_EP)) {
                try {
                    const se = device.getEndpoint(epId);
                    await se.read('genAnalogOutput', ['presentValue']);
                } catch (error) {
                    console.warn(`Tank setting endpoint ${epId} setup failed: ${error.message}`);
                }
            }
        },
        exposes: [
            e.binary('lockout', ea.ALL, 'ON', 'OFF')
                .withDescription('Pump lockout in AUTO mode; force modes remain available'),
            e.binary('relay', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Controller-owned pump relay output'),
            e.numeric('depth', ea.STATE_GET).withUnit('cm').withDescription('Water depth above sensor'),
            e.numeric('level', ea.STATE_GET).withUnit('%').withDescription('Tank level'),
            e.enum('fault', ea.STATE_GET, Object.values(FAULT_MAP).concat(['unknown']))
                .withDescription('Controller fault status'),
            e.binary('low_alert', ea.STATE_GET, 'ON', 'OFF')
                .withDescription('Low alert active (depth at/below the Low alert level)'),
            e.numeric('baro_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Barometric reference pressure'),
            e.numeric('tank_pressure', ea.STATE_GET).withUnit('hPa')
                .withDescription('Tank sensor absolute pressure'),
            e.numeric('air_temperature', ea.STATE_GET).withUnit('C')
                .withDescription('Air/reference sensor temperature'),
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
