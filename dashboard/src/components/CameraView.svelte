<script>
  // Live view from the gripper camera: the Pi's MJPEG stream, or in cloud mode the
  // snapshots the Pi sends over MQTT (snapshot is then an object URL, refreshed each second).
  let { status, snapshot = null, cloud = false } = $props();
  let failed = $state(false);
  let stream = $state('/stream.mjpg');
  const src = $derived(cloud ? snapshot : stream);

  function retry() {
    failed = false;
    stream = `/stream.mjpg?t=${Date.now()}`;
  }
</script>

<div class="camera">
  {#if cloud && !snapshot}
    <div class="retry">รอภาพจากหุ่นยนต์ผ่าน MQTT...</div>
  {:else if failed && !cloud}
    <button class="retry" onclick={retry}>ภาพกล้องขาดหาย แตะเพื่อลองใหม่</button>
  {:else}
    <img {src} alt="ภาพสดจากกล้องบนแขนกล" onerror={() => (failed = true)} />
  {/if}
  <span class="badge {status?.state}">{status ? `${status.state} · ${status.mode}` : 'กำลังเชื่อมต่อ...'}</span>
</div>

<style>
  .camera { position: relative; background: #000; border-radius: 14px; overflow: hidden; aspect-ratio: 4 / 3; }
  img { width: 100%; height: 100%; object-fit: contain; display: block; }
  .retry { display: grid; place-items: center; width: 100%; height: 100%; background: #000; color: var(--muted); border: 0; font-size: 1rem; }
  .badge { position: absolute; top: 8px; right: 8px; padding: 3px 10px; border-radius: 999px; font-size: 0.8rem; background: rgba(0, 0, 0, 0.6); }
  .badge.picking { background: var(--accent); color: #111; }
  .badge.resting { background: #5b6cff; }
</style>
