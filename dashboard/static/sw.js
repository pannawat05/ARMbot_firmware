const CACHE_NAME = 'armbot-dashboard-v1';
const APP_SHELL = [
  '/',
  '/manifest.webmanifest',
  '/favicon-16x16.png',
  '/favicon-32x32.png',
  '/apple-touch-icon.png',
  '/icon-192.png',
  '/icon-512.png',
];

self.addEventListener('install', (event) => {
  event.waitUntil(caches.open(CACHE_NAME).then((cache) => cache.addAll(APP_SHELL)).then(() => self.skipWaiting()));
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((key) => key.startsWith('armbot-dashboard-') && key !== CACHE_NAME).map((key) => caches.delete(key))),
    ),
  );
  event.waitUntil(self.clients.claim());
});

self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);
  if (request.method !== 'GET' || url.origin !== self.location.origin) return;

  if (request.mode === 'navigate') {
    event.respondWith(
      fetch(request)
        .then((response) => {
          const copy = response.clone();
          return caches.open(CACHE_NAME).then((cache) => cache.put('/', copy)).then(() => response);
        })
        .catch(async () => (await caches.match('/')) ?? Response.error()),
    );
    return;
  }

  if (url.pathname.startsWith('/assets/')) {
    event.respondWith(
      caches.match(request).then(
        (cached) =>
          cached ??
          fetch(request).then((response) => {
            if (response.ok) {
              const copy = response.clone();
              return caches.open(CACHE_NAME).then((cache) => cache.put(request, copy)).then(() => response);
            }
            return response;
          }),
      ),
    );
  }
});
