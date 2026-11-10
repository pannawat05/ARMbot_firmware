// How the dashboard talks to the robot.
//
// Over http:// the page uses the MQTT broker (settings from the Pi's /mqtt.json) through
// its WebSocket port. Over https:// browsers block a plain ws:// broker, so the page uses
// the Pi's own HTTP API instead (HTTPS is needed for the phone's mic and camera).
// If the broker cannot be reached within 5 s the page falls back to the HTTP API.
// Both give the same interface: status and item updates, plus queue / cancel / stop.

import * as api from './api.js';

export async function connect({ onStatus, onItems, onOnline }) {
  const cfg = location.protocol === 'http:' ? await fetch('/mqtt.json').then((r) => (r.ok ? r.json() : null)).catch(() => null) : null;
  if (cfg) {
    try {
      return await connectMqtt(cfg, { onStatus, onItems, onOnline });
    } catch (e) {
      console.warn(`MQTT ${cfg.ws_url} unreachable (${e.message}); using the Pi's HTTP API`);
    }
  }
  return connectHttp({ onStatus, onItems, onOnline });
}

const MQTT_CONNECT_MS = 5000;

async function connectMqtt(cfg, { onStatus, onItems, onOnline }) {
  const mod = await import('mqtt');
  const connect = mod.connect ?? mod.default.connect;
  const p = cfg.prefix;
  const client = connect(cfg.ws_url, {
    username: cfg.username,
    password: cfg.password,
    clientId: `armbot-web-${Math.random().toString(16).slice(2, 10)}`,
    reconnectPeriod: 2000,
    connectTimeout: MQTT_CONNECT_MS,
  });
  await new Promise((resolve, reject) => {
    const timer = setTimeout(() => (client.end(true), reject(new Error('timeout'))), MQTT_CONNECT_MS);
    client.once('connect', () => (clearTimeout(timer), resolve()));
  });
  const subscribe = () => client.subscribe([`${p}/status`, `${p}/items`, `${p}/online`]);
  subscribe(); // the first connect already happened above
  client.on('connect', subscribe); // and again after every reconnect
  client.on('close', () => onOnline(false));
  client.on('message', (topic, payload) => {
    const text = payload.toString();
    if (topic === `${p}/online`) return onOnline(text === '1');
    if (!text) return;
    const body = JSON.parse(text);
    if (topic === `${p}/status`) onStatus(body);
    if (topic === `${p}/items`) onItems(body.items);
  });
  const send = (cmd, body) =>
    new Promise((resolve, reject) =>
      client.publish(`${p}/cmd/${cmd}`, JSON.stringify(body), { qos: 1 }, (err) => (err ? reject(err) : resolve())),
    );
  return {
    mode: 'MQTT',
    queue: (order) => send('queue', order),
    cancel: (id) => send('cancel', { id }),
    stop: () => send('stop', {}),
    close: () => client.end(true),
  };
}

function connectHttp({ onStatus, onItems, onOnline }) {
  const status = () =>
    api.getStatus().then((s) => (onStatus(s), onOnline(true))).catch(() => onOnline(false));
  const items = () => api.getItems().then(onItems).catch(() => {});
  status();
  items();
  const timers = [setInterval(status, 1000), setInterval(items, 5000)];
  return {
    mode: 'HTTP',
    queue: (order) => api.queuePick(order).then(status),
    cancel: (id) => api.cancelJob(id).then(status),
    stop: () => api.stopRobot().then(status),
    close: () => timers.forEach(clearInterval),
  };
}
