<script>
  // Live view from the gripper camera (MJPEG from the server, with the pick overlay).
  let { status } = $props();
  let failed = $state(false);
  let src = $state('/stream.mjpg');

  function retry() {
    failed = false;
    src = `/stream.mjpg?t=${Date.now()}`;
  }
</script>

<div class="camera">
  {#if failed}
    <button class="retry" onclick={retry}>ภาพกล้องขาดหาย แตะเพื่อลองใหม่</button>
  {:else}
    <img {src} alt="ภาพสดจากกล้องบนแขนกล" onerror={() => (failed = true)} />
  {/if}
  <span class="badge {status?.state}">{status ? `${status.state} · ${status.mode}` : 'กำลังเชื่อมต่อ...'}</span>
</div>

<style>
  .camera { position: relative; background: #000; border-radius: 14px; overflow: hidden; aspect-ratio: 4 / 3; }
  img { width: 100%; height: 100%; object-fit: contain; display: block; }
  .retry { width: 100%; height: 100%; background: #000; color: var(--muted); border: 0; font-size: 1rem; }
  .badge { position: absolute; top: 8px; right: 8px; padding: 3px 10px; border-radius: 999px; font-size: 0.8rem; background: rgba(0, 0, 0, 0.6); }
  .badge.picking { background: var(--accent); color: #111; }
  .badge.resting { background: #5b6cff; }
</style>
