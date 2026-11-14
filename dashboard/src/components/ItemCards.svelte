<script>
  // Mobile: large cards. Set how many to pick with the stepper, then press the pick button.
  import QuantityInput from './QuantityInput.svelte';

  let { items, onpick } = $props();
  let counts = $state({}); // item id -> number to pick (1 when untouched)

  function order(item) {
    const n = Math.min(counts[item.id] ?? 1, Math.max(1, item.stock));
    counts[item.id] = 1;
    onpick(item, n);
  }
</script>

<div class="cards">
  {#each items as item (item.id)}
    <div class="card" class:empty={item.stock <= 0}>
      <span class="name">{item.name_th}</span>
      <span class="sub">{item.name}</span>
      <span class="meta"><b>{item.zone}</b> · คงเหลือ {item.stock}</span>
      <div class="order">
        <QuantityInput bind:value={counts[item.id]} max={item.stock} disabled={item.stock <= 0} />
        <button class="pick" onclick={() => order(item)} disabled={item.stock <= 0}>
          หยิบ{(counts[item.id] ?? 1) > 1 ? ` ${counts[item.id]} ชิ้น` : ''}
        </button>
      </div>
    </div>
  {/each}
</div>

<style>
  .cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(190px, 1fr)); gap: 10px; }
  .card { display: flex; flex-direction: column; align-items: flex-start; gap: 4px; min-height: 104px; padding: 14px; border-radius: 14px; border: 1px solid var(--line); background: var(--panel); text-align: left; }
  .card.empty { opacity: 0.55; }
  .name { font-size: 1.15rem; font-weight: 700; }
  .sub { color: var(--muted); font-size: 0.85rem; }
  .meta { font-size: 0.85rem; }
  .meta b { color: var(--accent); }
  .order { display: flex; gap: 8px; align-items: stretch; margin-top: auto; padding-top: 8px; width: 100%; }
  .pick { flex: 1; padding: 10px 12px; font-weight: 700; background: var(--accent); color: #111; border: 0; border-radius: 10px; }
  .pick:active { transform: scale(0.98); }
  .pick:disabled { opacity: 0.45; }
</style>
