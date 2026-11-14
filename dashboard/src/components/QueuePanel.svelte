<script>
  // Robot queue: running and waiting jobs first, then recent results.
  let { status, log = false, oncancel, onstop } = $props();
  const jobs = $derived(status?.jobs ?? []);
  const label = { queued: 'รอคิว', running: 'กำลังหยิบ', done: 'สำเร็จ', failed: 'ไม่สำเร็จ', cancelled: 'ยกเลิก' };
  const source = { click: 'กด', voice: 'เสียง', barcode: 'บาร์โค้ด' };
</script>

<div class="head">
  <h2>คิวหุ่นยนต์</h2>
  <button class="stop" onclick={onstop}>หยุด</button>
</div>

<ul>
  {#each jobs as job (job.id)}
    <li class={job.state}>
      <span class="chip">{label[job.state] ?? job.state}</span>
      <span class="name">{job.name_th}{job.count > 1 ? ` × ${job.count}` : ''}</span>
      <small>{source[job.source] ?? job.source}{job.count > 1 && (job.state === 'running' || job.state === 'queued') ? ` · หยิบแล้ว ${job.done}/${job.count}` : ''}{job.message ? ` · ${job.message}` : ''}</small>
      {#if job.state === 'queued'}
        <button class="x" onclick={() => oncancel(job.id)} aria-label="ยกเลิก">✕</button>
      {/if}
    </li>
  {:else}
    <li class="empty">ยังไม่มีงาน</li>
  {/each}
</ul>

{#if log && status?.log?.length}
  <details>
    <summary>บันทึกการทำงาน</summary>
    <pre>{status.log.join('\n')}</pre>
  </details>
{/if}

<style>
  .head { display: flex; justify-content: space-between; align-items: center; }
  h2 { margin: 0; font-size: 1.05rem; }
  .stop { background: var(--bad); color: #fff; border: 0; padding: 8px 18px; font-weight: 700; }
  ul { list-style: none; margin: 8px 0 0; padding: 0; display: flex; flex-direction: column; gap: 6px; }
  li { display: grid; grid-template-columns: auto 1fr auto; grid-template-rows: auto auto; column-gap: 8px; align-items: center; padding: 8px 10px; border-radius: 10px; background: var(--panel); border: 1px solid var(--line); }
  li small { grid-column: 2; color: var(--muted); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .chip { grid-row: span 2; font-size: 0.75rem; padding: 2px 8px; border-radius: 999px; background: var(--muted-bg); }
  .running .chip { background: var(--accent); color: #111; }
  .done .chip { background: var(--ok); color: #04210f; }
  .failed .chip { background: var(--bad); color: #fff; }
  .cancelled { opacity: 0.55; }
  .x { grid-row: span 2; grid-column: 3; background: transparent; border: 0; color: var(--muted); font-size: 1rem; }
  .empty { display: block; color: var(--muted); text-align: center; }
  pre { max-height: 220px; overflow: auto; font-size: 0.75rem; background: #000; padding: 8px; border-radius: 8px; white-space: pre-wrap; }
</style>
