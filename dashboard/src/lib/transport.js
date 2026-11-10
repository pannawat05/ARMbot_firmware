// How the dashboard talks to the robot.
//
// The page uses the MQTT broker (settings from the Pi's /mqtt.json) through its WebSocket
// port. Over https:// browsers block a plain ws:// broker, so with a ws:// broker the page
// uses the Pi's own HTTP API instead (HTTPS is needed for the phone's mic and camera);
// a wss:// broker works from both.
// If the broker cannot be reached within 5 s the page falls back to the HTTP API.
//
// Cloud mode: when the page is served from somewhere other than the Pi (the nginx image in
// Dockerfile.cloud), /config.json carries the broker settings. The page then uses MQTT only,
// keeps retrying, and shows the camera from the Pi's <prefix>/camera snapshots.
// Both give the same interface: status and item updates, plus queue / cancel / stop.

import * as api from './api.js';

const getJson = (url) => fetch(url).then((r) => (r.ok ? r.json() : null)).catch(() => null);

export async function connect({ onStatus, onItems, onOnline, onCamera }) {
  const cloud = await getJson('/config.json');
  if (cloud?.mqtt) {
    const link = await connectMqtt(cloud.mqtt, { onStatus, onItems, onOnline, onCamera }, false);
    return { ...link, mode: 'MQTT cloud', cloud: true };
  }
  const cfg = await getJson('/mqtt.json');
  if (cfg?.ws_url && (location.protocol === 'http:' || !cfg.ws_url.startsWith('ws:'))) {
    try {
      return await connectMqtt(cfg, { onStatus, onItems, onOnline });
    } catch (e) {
      console.warn(`MQTT ${cfg.ws_url} unreachable (${e.message}); using the Pi's HTTP API`);
    }
  }
  return connectHttp({ onStatus, onItems, onOnline });
}

const MQTT_CONNECT_MS = 5000;

async function connectMqtt(cfg, { onStatus, onItems, onOnline, onCamera }, mustConnect = true) {
  const mod = await import('mqtt');
  const connect = mod.connect ?? mod.default.connect;
  const p = cfg.prefix;
  // "/mqtt" means this page's own server, which forwards to the broker (cloud image).
  const url = cfg.ws_url.startsWith('/')
    ? `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}${cfg.ws_url}`
    : cfg.ws_url;
  const client = connect(url, {
    username: cfg.username || undefined, // public brokers take no login
    password: cfg.password || undefined,
    clientId: `armbot-web-${Math.random().toString(16).slice(2, 10)}`,
    reconnectPeriod: 2000,
    connectTimeout: MQTT_CONNECT_MS,
  });
  const topics = [`${p}/status`, `${p}/items`, `${p}/online`, ...(onCamera ? [`${p}/camera`] : [])];
  const subscribe = () => client.subscribe(topics);
  if (mustConnect) {
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => (client.end(true), reject(new Error('timeout'))), MQTT_CONNECT_MS);
      client.once('connect', () => (clearTimeout(timer), resolve()));
    });
    subscribe(); // the first connect already happened above
  }
  client.on('connect', subscribe); // and again after every reconnect
  let picture;
  client.on('close', () => onOnline(false));
  client.on('message', (topic, payload) => {
    if (topic === `${p}/camera`) {
      if (picture) URL.revokeObjectURL(picture);
      picture = URL.createObjectURL(new Blob([payload], { type: 'image/jpeg' }));
      return onCamera(picture);
    }
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
