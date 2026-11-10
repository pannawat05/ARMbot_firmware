<script>
  import { onDestroy, onMount } from 'svelte';
  import { connect } from './lib/transport.js';
  import { createVoice } from './lib/voice.svelte.js';
  import { beep } from './lib/feedback.js';
  import VoiceBar from './components/VoiceBar.svelte';
  import CameraView from './components/CameraView.svelte';
  import ItemCards from './components/ItemCards.svelte';
  import ItemTable from './components/ItemTable.svelte';
  import QueuePanel from './components/QueuePanel.svelte';
  import BarcodeScanner from './components/BarcodeScanner.svelte';

  let items = $state([]);
  let status = $state(null);
  let offline = $state(false);
  let link = $state(null); // MQTT or HTTP connection to the robot
  let camera = $state(null); // cloud mode: latest snapshot from the Pi
  let toast = $state('');
  let scanning = $state(false);
  let scanOnly = $state(false);
  let lastScannedCode = $state('');
  let search = $state('');
  let theme = $state(document.documentElement.dataset.theme === 'light' ? 'light' : 'dark');
  const media = window.matchMedia('(max-width: 820px)');
  let mobile = $state(media.matches);
  media.addEventListener('change', (e) => (mobile = e.matches));

  function toggleTheme() {
    theme = theme === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = theme;
    document.querySelector('meta[name="theme-color"]')?.setAttribute('content', theme === 'dark' ? '#0f1115' : '#f3f5f8');
    try {
      localStorage.setItem('armbot-theme', theme);
    } catch (error) {
      console.warn('Could not save theme preference:', error);
    }
  }

  const findItem = (text) => {
    const t = text.toLowerCase();
    return items.find((i) => i.aliases.some((a) => t.includes(a.toLowerCase())) || i.barcode === text.trim());
  };

  function show(message) {
    toast = message;
    setTimeout(() => toast === message && (toast = ''), 3500);
  }

  async function order(o) {
    const item = o.item_id ? items.find((i) => i.id === o.item_id) : o.barcode ? items.find((i) => i.barcode === o.barcode) : findItem(o.text ?? '');
    if (!item) throw new Error('ไม่พบของชิ้นนี้');
    await link.queue({ item_id: item.id, source: o.source });
    show(`เพิ่ม ${item.name_th} เข้าคิวแล้ว`);
    return { item };
  }

  const pick = (item) => order({ item_id: item.id, source: 'click' }).catch((e) => show(`สั่งไม่สำเร็จ: ${e.message}`));

  function byCode(code) {
    beep(1200, 80);
    order({ barcode: code, source: 'barcode' }).catch(() => show(`ไม่พบบาร์โค้ด ${code}`));
  }

  function startScan(testOnly = false) {
    scanOnly = testOnly;
    scanning = true;
  }

  function onBarcodeScanned(code) {
    scanning = false;
    beep(1200, 80);
    if (scanOnly) {
      lastScannedCode = code;
      return;
    }
    order({ barcode: code, source: 'barcode' }).catch(() => show(`ไม่พบบาร์โค้ด ${code}`));
  }

  function submitSearch(e) {
    e.preventDefault();
    const q = search.trim();
    if (!q) return;
    const item = findItem(q) ?? items.find((i) => i.name.toLowerCase().includes(q.toLowerCase()) || i.name_th.includes(q));
    if (item) pick(item);
    else show(`ไม่พบ "${q}"`);
    search = '';
  }

  const voice = createVoice({ findItem, order });

  // USB/Bluetooth scanners type the code fast and press Enter. Catch that anywhere except text fields.
  let buffer = '';
  let last = 0;
  function onKey(e) {
    if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
    const now = performance.now();
    if (now - last > 80) buffer = '';
    last = now;
    if (e.key === 'Enter' && buffer.length >= 3) {
      byCode(buffer);
      buffer = '';
    } else if (e.key.length === 1) buffer += e.key;
  }

  onMount(async () => {
    if ('serviceWorker' in navigator && !import.meta.env.DEV) {
      navigator.serviceWorker.register('/sw.js').catch((error) => console.warn('Could not register service worker:', error));
    }
    window.addEventListener('keydown', onKey);
    link = await connect({
      onStatus: (s) => (status = s),
      onItems: (list) => (items = list),
      onOnline: (up) => (offline = !up),
      onCamera: (url) => (camera = url),
    });
  });
  onDestroy(() => {
    link?.close();
    window.removeEventListener('keydown', onKey);
    voice.stop();
  });
</script>

{#if offline}<div class="offline">เชื่อมต่อหุ่นยนต์ไม่ได้{link ? ` (${link.mode})` : ''}</div>{/if}

{#if mobile}
  <main class="mobile">
    <section class="top">
      <header>
        <h1>Armbot</h1>
        {#if link}<span class="link">{link.mode}</span>{/if}
        <button class="theme-toggle" onclick={toggleTheme} aria-label={theme === 'dark' ? 'เปลี่ยนเป็นโหมดสว่าง' : 'เปลี่ยนเป็นโหมดมืด'}>
          {theme === 'dark' ? '☀️ โหมดสว่าง' : '🌙 โหมดมืด'}
        </button>
      </header>
      <CameraView {status} snapshot={camera} cloud={link?.cloud} />
      <VoiceBar {voice} compact />
    </section>
    <section class="bottom">
      <div class="scan-actions">
        <button class="scan" onclick={() => startScan()}>สแกนบาร์โค้ดเพื่อสั่งหยิบ</button>
        <button onclick={() => startScan(true)}>ทดลองสแกน (ไม่สั่งแขนกล)</button>
      </div>
      {#if lastScannedCode}
        <p class="scan-result">บาร์โค้ดที่อ่านได้: <code>{lastScannedCode}</code></p>
      {/if}
      <ItemCards {items} onpick={pick} />
      <QueuePanel {status} oncancel={(id) => link?.cancel(id)} onstop={() => link?.stop()} />
    </section>
  </main>
{:else}
  <main class="desktop">
    <aside>
      <header>
        <h1>Armbot</h1>
        {#if link}<span class="link">{link.mode}</span>{/if}
        <button class="theme-toggle" onclick={toggleTheme} aria-label={theme === 'dark' ? 'เปลี่ยนเป็นโหมดสว่าง' : 'เปลี่ยนเป็นโหมดมืด'}>
          {theme === 'dark' ? '☀️ โหมดสว่าง' : '🌙 โหมดมืด'}
        </button>
      </header>
      <CameraView {status} snapshot={camera} cloud={link?.cloud} />
      <VoiceBar {voice} />
      <QueuePanel {status} log oncancel={(id) => link?.cancel(id)} onstop={() => link?.stop()} />
    </aside>
    <section class="items">
      <button class="scan-test" onclick={() => startScan(true)}>ทดลองสแกนบาร์โค้ด (ไม่สั่งแขนกล)</button>
      {#if lastScannedCode}
        <p class="scan-result">บาร์โค้ดที่อ่านได้: <code>{lastScannedCode}</code></p>
      {/if}
      <form class="search" onsubmit={submitSearch}>
        <input placeholder="ค้นหาแล้วกด Enter เพื่อสั่งหยิบ หรือยิงบาร์โค้ด" bind:value={search} />
        <button type="submit">สั่งหยิบ</button>
      </form>
      <ItemTable {items} onpick={pick} />
    </section>
  </main>
{/if}

{#if scanning}
  <BarcodeScanner {scanOnly} onscan={onBarcodeScanned} onclose={() => (scanning = false)} />
{/if}

{#if toast}<div class="toast">{toast}</div>{/if}

<style>
  .mobile { display: flex; flex-direction: column; min-height: 100dvh; }
  .mobile .top { position: sticky; top: 0; z-index: 2; display: flex; flex-direction: column; gap: 8px; padding: 10px 12px; background: var(--bg); }
  .mobile .bottom { display: flex; flex-direction: column; gap: 12px; padding: 0 12px 24px; }
  .scan-actions { display: grid; gap: 8px; }
  .scan { padding: 16px; font-size: 1.1rem; font-weight: 700; background: var(--accent); color: #111; border: 0; }
  .scan-result { margin: 0; padding: 10px 12px; border-radius: 10px; background: var(--panel); overflow-wrap: anywhere; }
  .scan-result code { color: var(--accent); }
  .desktop { display: grid; grid-template-columns: minmax(340px, 420px) 1fr; gap: 20px; padding: 20px; min-height: 100vh; }
  aside { display: flex; flex-direction: column; gap: 12px; }
  header { display: flex; align-items: center; gap: 10px; }
  h1 { margin: 0; font-size: 1.3rem; }
  .theme-toggle { margin-left: auto; white-space: nowrap; }
  .link { font-size: 0.75rem; padding: 2px 8px; border-radius: 999px; background: var(--muted-bg); color: var(--muted); }
  .search { display: flex; gap: 8px; margin-bottom: 14px; }
  .search input { flex: 1; font-size: 1rem; }
  .scan-test { margin-bottom: 12px; }
  .offline { background: var(--bad); color: #fff; text-align: center; padding: 6px; }
  .toast { position: fixed; left: 50%; bottom: 20px; transform: translateX(-50%); background: var(--panel-hi); color: var(--text); backdrop-filter: blur(6px); border: 1px solid var(--line); padding: 10px 16px; border-radius: 12px; z-index: 20; }
</style>
