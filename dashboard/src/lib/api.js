// Calls to python/armbot_server.py on the same origin.

async function request(method, path, body) {
  const res = await fetch(path, {
    method,
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw Object.assign(new Error(data.error || res.statusText), { status: res.status, data });
  return data;
}

export const getItems = () => request('GET', '/api/items').then((d) => d.items);
export const getStatus = () => request('GET', '/api/status');
/** order: { item_id } | { barcode } | { text }, plus source: 'click' | 'voice' | 'barcode' */
export const queuePick = (order) => request('POST', '/api/queue', order);
export const cancelJob = (id) => request('DELETE', `/api/queue/${id}`);
export const stopRobot = () => request('POST', '/api/stop');
