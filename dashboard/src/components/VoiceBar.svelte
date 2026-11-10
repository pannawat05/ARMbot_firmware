<script>
  // Voice status: grey mic in standby, green with a waveform while listening for a command.
  let { voice, compact = false } = $props();
  const s = $derived(voice.state);
</script>

<div class="voice {s.mode}" class:compact>
  <button class="mic" onclick={voice.toggle} disabled={!s.supported} aria-label="เปิด/ปิดการสั่งด้วยเสียง">
    <svg viewBox="0 0 24 24" width="26" height="26" aria-hidden="true">
      <path fill="currentColor" d="M12 14a3 3 0 0 0 3-3V5a3 3 0 0 0-6 0v6a3 3 0 0 0 3 3Zm5-3a5 5 0 0 1-10 0H5a7 7 0 0 0 6 6.92V21h2v-3.08A7 7 0 0 0 19 11h-2Z" />
    </svg>
  </button>
  <div class="text">
    {#if !s.supported}
      <strong>เบราว์เซอร์นี้ไม่รองรับการสั่งด้วยเสียง</strong>
      <span>ใช้ Google Chrome บน Android หรือบนคอม</span>
    {:else if !s.secure}
      <strong>สั่งด้วยเสียงต้องเปิดหน้านี้ผ่าน https://</strong>
      <span class="error">หน้านี้เป็น http:// เบราว์เซอร์จึงไม่ให้ใช้ไมค์</span>
    {:else}
      <strong>{s.message}</strong>
      {#if s.mode === 'active'}
        <span class="wave" aria-hidden="true"><i></i><i></i><i></i><i></i><i></i></span>
      {/if}
      {#if s.heard && s.mode !== 'off'}<span class="heard">“{s.heard}”</span>{/if}
      {#if s.error}<span class="error">{s.error}</span>{/if}
    {/if}
  </div>
</div>

<style>
  .voice { display: flex; gap: 12px; align-items: center; padding: 10px 12px; border-radius: 14px; background: var(--panel); border: 1px solid var(--line); }
  .mic { flex: none; width: 52px; height: 52px; border-radius: 50%; border: 0; background: var(--muted-bg); color: var(--muted); }
  .standby .mic { background: #2a2f3a; color: #c9ced8; }
  .active .mic { background: var(--ok); color: #04210f; box-shadow: 0 0 0 6px color-mix(in srgb, var(--ok) 25%, transparent); }
  .confirm .mic { background: var(--accent); color: #111; }
  .text { display: flex; flex-direction: column; gap: 2px; min-width: 0; }
  .text strong { font-size: 1rem; }
  .heard { color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .error { color: var(--bad); }
  .wave { display: inline-flex; gap: 3px; height: 16px; align-items: center; }
  .wave i { width: 4px; height: 6px; background: var(--ok); border-radius: 2px; animation: wave 0.9s infinite ease-in-out; }
  .wave i:nth-child(2) { animation-delay: 0.1s; } .wave i:nth-child(3) { animation-delay: 0.2s; }
  .wave i:nth-child(4) { animation-delay: 0.3s; } .wave i:nth-child(5) { animation-delay: 0.4s; }
  @keyframes wave { 50% { height: 16px; } }
  .compact .mic { width: 44px; height: 44px; }
</style>
