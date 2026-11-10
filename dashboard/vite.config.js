import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

// npm run dev: the dashboard on :5173, talking to armbot_server.py on :8443.
export default defineConfig({
  plugins: [svelte()],
  server: {
    proxy: {
      '/api': { target: 'https://localhost:8443', secure: false },
      '/stream.mjpg': { target: 'https://localhost:8443', secure: false },
    },
  },
});
