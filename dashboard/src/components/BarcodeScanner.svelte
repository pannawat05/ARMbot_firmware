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
      <div class="viewfinder">
        <!-- svelte-ignore a11y_media_has_caption -->
        <video bind:this={video} playsinline muted></video>
        <div class="aim-frame" aria-hidden="true"></div>
      </div>
      <p>{scanOnly ? 'จัดบาร์โค้ดให้อยู่ในกรอบกลาง โหมดทดลองจะไม่ส่งคำสั่งไปยังแขนกล' : 'จัดบาร์โค้ดให้อยู่ในกรอบกลางเพื่อสแกน'}</p>
    {/if}
    <button onclick={onclose}>ปิด</button>
  </div>
</div>

<style>
  .overlay { position: fixed; inset: 0; background: rgba(0, 0, 0, 0.85); display: grid; place-items: center; z-index: 10; padding: 16px; }
  .box { width: min(480px, 100%); display: flex; flex-direction: column; gap: 10px; text-align: center; }
  .viewfinder { position: relative; overflow: hidden; border-radius: 14px; background: #000; }
  video { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: cover; }
  .aim-frame {
    position: absolute;
    left: 10%;
    top: 35%;
    width: 80%;
    height: 30%;
    border: 2px solid #ffb020;
    border-radius: 10px;
    box-shadow: 0 0 0 999px rgb(0 0 0 / 22%), 0 0 12px rgb(0 0 0 / 55%);
    pointer-events: none;
  }
  .aim-frame::before,
  .aim-frame::after {
    position: absolute;
    left: 50%;
    top: 50%;
    display: block;
    content: '';
    background: #ffb020;
    transform: translate(-50%, -50%);
  }
  .aim-frame::before { width: 18px; height: 2px; }
  .aim-frame::after { width: 2px; height: 18px; }
  .error { color: var(--bad); }
  button { padding: 12px; font-size: 1rem; }
</style>
