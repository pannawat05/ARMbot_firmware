<script>
  // Mobile: large cards, one tap orders a pick.
  let { items, onpick } = $props();
</script>

<div class="cards">
  {#each items as item (item.id)}
    <button class="card" onclick={() => onpick(item)} disabled={item.stock <= 0}>
      <span class="name">{item.name_th}</span>
      <span class="sub">{item.name}</span>
      <span class="meta"><b>{item.zone}</b> · คงเหลือ {item.stock}</span>
    </button>
  {/each}
</div>

<style>
  .cards { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr)); gap: 10px; }
  .card { display: flex; flex-direction: column; align-items: flex-start; gap: 4px; min-height: 104px; padding: 14px; border-radius: 14px; border: 1px solid var(--line); background: var(--panel); color: inherit; text-align: left; }
  .card:active { transform: scale(0.98); background: var(--panel-hi); }
  .card:disabled { opacity: 0.45; }
  .name { font-size: 1.15rem; font-weight: 700; }
  .sub { color: var(--muted); font-size: 0.85rem; }
  .meta { margin-top: auto; font-size: 0.85rem; }
  .meta b { color: var(--accent); }
</style>
