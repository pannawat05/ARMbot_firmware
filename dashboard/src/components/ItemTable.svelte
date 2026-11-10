<script>
  // Desktop: every item with stock, zone and pick count; filter by text or zone.
  let { items, onpick } = $props();
  let query = $state('');
  let zone = $state('');
  const zones = $derived([...new Set(items.map((i) => i.zone))].sort());
  const shown = $derived(
    items.filter((i) => {
      const q = query.trim().toLowerCase();
      const text = [i.name, i.name_th, i.barcode, i.zone, ...i.aliases].join(' ').toLowerCase();
      return (!q || text.includes(q)) && (!zone || i.zone === zone);
    }),
  );
</script>

<div class="filters">
  <input placeholder="กรองชื่อ / บาร์โค้ด" bind:value={query} />
  <select bind:value={zone} aria-label="โซน">
    <option value="">ทุกโซน</option>
    {#each zones as z}<option>{z}</option>{/each}
  </select>
</div>

<table>
  <thead>
    <tr><th>ชื่อ</th><th>โซน</th><th class="num">คงเหลือ</th><th class="num">ถูกหยิบ</th><th>บาร์โค้ด</th><th></th></tr>
  </thead>
  <tbody>
    {#each shown as item (item.id)}
      <tr>
        <td><b>{item.name_th}</b><br /><small>{item.name}</small></td>
        <td>{item.zone}</td>
        <td class="num" class:low={item.stock <= 1}>{item.stock}</td>
        <td class="num">{item.picks}</td>
        <td><code>{item.barcode}</code></td>
        <td><button onclick={() => onpick(item)} disabled={item.stock <= 0}>หยิบ</button></td>
      </tr>
    {:else}
      <tr><td colspan="6" class="empty">ไม่พบรายการ</td></tr>
    {/each}
  </tbody>
</table>

<style>
  .filters { display: flex; gap: 8px; margin-bottom: 10px; }
  .filters input { flex: 1; }
  table { width: 100%; border-collapse: collapse; }
  th, td { padding: 10px 8px; border-bottom: 1px solid var(--line); text-align: left; vertical-align: middle; }
  th { color: var(--muted); font-weight: 500; font-size: 0.85rem; }
  .num { text-align: right; font-variant-numeric: tabular-nums; }
  .low { color: var(--bad); font-weight: 700; }
  small { color: var(--muted); }
  .empty { text-align: center; color: var(--muted); }
  button { padding: 8px 16px; }
</style>
