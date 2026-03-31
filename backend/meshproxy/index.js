require('dotenv').config();

const fs = require('fs');
const path = require('path');
const mqtt = require('mqtt');
const { SerialPort } = require('serialport');
const protobuf = require('protobufjs');

const START1 = 0x94;
const START2 = 0xc3;

const SERIAL_PORT = process.env.SERIAL_PORT || '/dev/ttyACM0';
const SERIAL_BAUD = Number(process.env.SERIAL_BAUD || 38400);
const MQTT_URL = process.env.MQTT_URL || 'mqtt://localhost:1883';
const MQTT_USERNAME = process.env.MQTT_USERNAME || undefined;
const MQTT_PASSWORD = process.env.MQTT_PASSWORD || undefined;
const MQTT_TOPIC_PREFIX = process.env.MQTT_TOPIC_PREFIX || 'wishmesh';
const FORWARD_RAW_BASE64 = String(process.env.FORWARD_RAW_BASE64 || 'true').toLowerCase() === 'true';

function publishJson(client, topic, obj) {
  client.publish(topic, JSON.stringify(obj), { qos: 0, retain: false });
}

async function loadProto() {
  const protoRoot = path.join(__dirname, '..', '..', 'third_party', 'meshtastic', 'protobufs');
  const files = [
    path.join(protoRoot, 'meshtastic', 'mesh.proto'),
    path.join(protoRoot, 'meshtastic', 'portnums.proto'),
    path.join(protoRoot, 'meshtastic', 'channel.proto'),
  ].filter(fs.existsSync);

  const root = new protobuf.Root();
  root.resolvePath = function(origin, target) {
    if (fs.existsSync(target)) return target;
    const p1 = path.join(protoRoot, target);
    if (fs.existsSync(p1)) return p1;
    const p2 = path.join(protoRoot, 'meshtastic', path.basename(target));
    if (fs.existsSync(p2)) return p2;
    return target;
  };

  for (const file of files) {
    await root.load(file, { keepCase: true });
  }
  root.resolveAll();
  return root;
}

function createFrameDecoder(onFrame) {
  let buf = Buffer.alloc(0);

  return (chunk) => {
    buf = Buffer.concat([buf, chunk]);

    while (buf.length >= 4) {
      let start = -1;
      for (let i = 0; i < buf.length - 1; i++) {
        if (buf[i] === START1 && buf[i + 1] === START2) {
          start = i;
          break;
        }
      }

      if (start < 0) {
        buf = Buffer.alloc(0);
        return;
      }

      if (start > 0) {
        buf = buf.slice(start);
      }

      if (buf.length < 4) return;

      const len = buf.readUInt16BE(2);
      const frameLen = 4 + len;
      if (buf.length < frameLen) return;

      const payload = buf.slice(4, frameLen);
      buf = buf.slice(frameLen);
      onFrame(payload);
    }
  };
}

function decodePacket(root, payload) {
  const candidates = [
    'meshtastic.FromRadio',
    'FromRadio',
  ];

  for (const name of candidates) {
    const type = root.lookupType(name, false);
    if (!type) continue;
    try {
      const decoded = type.decode(payload);
      return { typeName: name, decoded };
    } catch (_) {}
  }

  return null;
}

function maybeExtractText(decodedObj) {
  const obj = decodedObj && decodedObj.toJSON ? decodedObj.toJSON() : decodedObj;
  if (!obj || typeof obj !== 'object') return null;

  // Common Meshtastic shape: packet.decoded.text or packet.decoded.payload
  const packet = obj.packet || obj.rxPacket || obj.meshPacket || null;
  if (!packet) return null;

  const decoded = packet.decoded || {};

  if (typeof decoded.text === 'string' && decoded.text.length) {
    return {
      from: packet.from,
      to: packet.to,
      id: packet.id,
      channel: packet.channel,
      portnum: decoded.portnum,
      text: decoded.text,
    };
  }

  if (decoded.payload) {
    try {
      const raw = Buffer.from(decoded.payload, 'base64');
      const text = raw.toString('utf8').replace(/\0+$/, '');
      if (text && /^[\x09\x0A\x0D\x20-\x7E\u00A0-\u024F]+$/.test(text)) {
        return {
          from: packet.from,
          to: packet.to,
          id: packet.id,
          channel: packet.channel,
          portnum: decoded.portnum,
          text,
        };
      }
    } catch (_) {}
  }

  return null;
}

async function main() {
  const root = await loadProto();
  const mqttClient = mqtt.connect(MQTT_URL, {
    username: MQTT_USERNAME,
    password: MQTT_PASSWORD,
  });

  mqttClient.on('connect', () => {
    console.log(`[mqtt] connected to ${MQTT_URL}`);
    publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/status`, {
      ts: new Date().toISOString(),
      state: 'connected',
      serialPort: SERIAL_PORT,
      serialBaud: SERIAL_BAUD,
    });
  });

  mqttClient.on('error', (err) => {
    console.error('[mqtt] error:', err.message);
  });

  const port = new SerialPort({
    path: SERIAL_PORT,
    baudRate: SERIAL_BAUD,
    autoOpen: false,
  });

  const decoder = createFrameDecoder((framePayload) => {
    const ts = new Date().toISOString();
    const parsed = decodePacket(root, framePayload);

    if (!parsed) {
      if (FORWARD_RAW_BASE64) {
        publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/raw`, {
          ts,
          kind: 'unknown-frame',
          payload_b64: framePayload.toString('base64'),
        });
      }
      return;
    }

    const textMsg = maybeExtractText(parsed.decoded);
    if (textMsg) {
      publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/text`, {
        ts,
        ...textMsg,
      });
      console.log('[mesh:text]', textMsg);
      return;
    }

    if (FORWARD_RAW_BASE64) {
      publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/raw`, {
        ts,
        protobuf_type: parsed.typeName,
        decoded: parsed.decoded.toJSON ? parsed.decoded.toJSON() : parsed.decoded,
        payload_b64: framePayload.toString('base64'),
      });
    }
  });

  port.on('data', decoder);
  port.on('error', (err) => {
    console.error('[serial] error:', err.message);
    publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/status`, {
      ts: new Date().toISOString(),
      state: 'serial-error',
      error: err.message,
    });
  });

  port.on('open', () => {
    console.log(`[serial] open ${SERIAL_PORT} @ ${SERIAL_BAUD}`);
    publishJson(mqttClient, `${MQTT_TOPIC_PREFIX}/status`, {
      ts: new Date().toISOString(),
      state: 'serial-open',
      serialPort: SERIAL_PORT,
      serialBaud: SERIAL_BAUD,
    });
  });

  port.open((err) => {
    if (err) {
      console.error('[serial] open failed:', err.message);
      process.exitCode = 1;
    }
  });
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
