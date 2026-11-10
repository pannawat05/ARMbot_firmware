<script>
  // Phone camera barcode scan with the browser's BarcodeDetector (Chrome on Android).
  import { onDestroy, onMount } from 'svelte';

  let { onscan, onclose, scanOnly = false } = $props();
  let video;
  let stream;
  let error = $state('');
  let running = true;

  onMount(async () => {
    if (!('BarcodeDetector' in window)) {
      error = 'เบราว์เซอร์นี้ไม่รองรับการสแกนบาร์โค้ดด้วยกล้อง (ใช้ Chrome บน Android)';
      return;
    }
    try {
      stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'environment' } });
      video.srcObject = stream;
      await video.play();
      const detector = new window.BarcodeDetector();
      while (running) {
        const codes = await detector.detect(video).catch(() => []);
        if (codes.length) {
          navigator.vibrate?.(80);
          onscan(codes[0].rawValue);
          return;
        }
        await new Promise((r) => setTimeout(r, 200));
      }
    } catch (e) {
      error = e.name === 'NotAllowedError' ? 'ไม่ได้รับอนุญาตให้ใช้กล้อง (ต้องเปิดผ่าน HTTPS)' : `เปิดกล้องไม่ได้: ${e.message}`;
    }
  });

  onDestroy(() => {
    running = false;
    stream?.getTracks().forEach((t) => t.stop());
  });
</script>

<div class="overlay" role="dialog" aria-label="สแกนบาร์โค้ด">
  <div class="box">
    {#if error}
      <p class="error">{error}</p>
    {:else}
      <!-- svelte-ignore a11y_media_has_caption -->
      <video bind:this={video} playsinline muted></video>
      <p>{scanOnly ? 'โหมดทดลอง: สแกนแล้วจะแสดงบาร์โค้ดเท่านั้น ไม่ส่งคำสั่งไปยังแขนกล' : 'เล็งกล้องไปที่บาร์โค้ดของชิ้นงาน'}</p>
    {/if}
    <button onclick={onclose}>ปิด</button>
  </div>
</div>

<style>
  .overlay { position: fixed; inset: 0; background: rgba(0, 0, 0, 0.85); display: grid; place-items: center; z-index: 10; padding: 16px; }
  .box { width: min(480px, 100%); display: flex; flex-direction: column; gap: 10px; text-align: center; }
  video { width: 100%; border-radius: 14px; background: #000; }
  .error { color: var(--bad); }
  button { padding: 12px; font-size: 1rem; }
</style>
