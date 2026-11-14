<script>
  // Number of items to pick, set before pressing the pick button: - [n] +, between 1 and the stock.
  // value has no default: the parent binds an entry that is undefined until first used, and
  // Svelte refuses to bind undefined to a prop that has a fallback. n is what is shown.
  let { value = $bindable(), max = 1, disabled = false } = $props();
  const n = $derived(value ?? 1);
  const clamp = (x) => Math.max(1, Math.min(Number.isFinite(x) ? Math.round(x) : 1, Math.max(1, max)));
  const set = (x) => (value = clamp(x));
  // The stock can drop while the page is open: keep the number inside it.
  $effect(() => {
    if (value !== undefined && value > Math.max(1, max)) value = clamp(value);
  });
</script>

<div class="qty" class:disabled>
  <button type="button" onclick={() => set(n - 1)} disabled={disabled || n <= 1} aria-label="ลดจำนวน">−</button>
  <input
    type="number"
    inputmode="numeric"
    min="1"
    {max}
    {disabled}
    value={n}
    onchange={(e) => set(e.currentTarget.valueAsNumber)}
    aria-label="จำนวนที่จะหยิบ"
  />
  <button type="button" onclick={() => set(n + 1)} disabled={disabled || n >= max} aria-label="เพิ่มจำนวน">+</button>
</div>

<style>
  .qty { display: inline-flex; align-items: stretch; border: 1px solid var(--line); border-radius: 10px; overflow: hidden; background: var(--panel); }
  .qty.disabled { opacity: 0.45; }
  button { width: 38px; padding: 0; border: 0; border-radius: 0; background: var(--muted-bg); color: inherit; font-size: 1.2rem; line-height: 1; }
  button:disabled { opacity: 0.4; }
  input { width: 48px; padding: 6px 0; border: 0; border-radius: 0; text-align: center; font-size: 1.05rem; font-variant-numeric: tabular-nums; background: transparent; color: inherit; -moz-appearance: textfield; appearance: textfield; }
  input::-webkit-outer-spin-button, input::-webkit-inner-spin-button { -webkit-appearance: none; margin: 0; }
</style>
