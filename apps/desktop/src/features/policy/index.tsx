import { defineComponent, onMounted, ref, shallowRef } from "vue";
import { useEventListener } from "@vueuse/core";
import { desktopBackend, useDesktopStore } from "../../store";
import style from "./style.module.scss";

export const PolicyPage = defineComponent(() => {
  const store = useDesktopStore();
  const busy = ref<string | null>(null);
  const error = ref<string | null>(null);
  const refreshButton = shallowRef<HTMLButtonElement>();

  const refresh = async () => {
    try {
      store.state.policies = await desktopBackend().policyList();
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const download = async (id: string) => {
    busy.value = id;
    try {
      await desktopBackend().policyDownload(id);
      await refresh();
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      busy.value = null;
    }
  };

  onMounted(() => {
    if (!store.state.policies.length && !store.state.loading) void refresh();
  });
  useEventListener(refreshButton, "click", () => void refresh());
  const PolicyDownloadButton = defineComponent({
    props: {
      policyId: { type: String, required: true },
    },
    setup(props) {
      const button = shallowRef<HTMLButtonElement>();
      useEventListener(button, "click", () => void download(props.policyId));
      return () => (
        <button ref={button} type="button" disabled={busy.value !== null} class={style.primary}>
          {busy.value === props.policyId ? "下载中…" : "下载"}
        </button>
      );
    },
  });

  return () => (
    <div class={[style.page, style.policyPage]}>
      <header class={style.policyHeader}>
        <div>
          <h1>策略</h1>
          <p>管理当前电脑缓存的官方 ONNX 文件。</p>
        </div>
        <button ref={refreshButton} type="button" disabled={busy.value !== null}>
          刷新
        </button>
      </header>

      <section class={style.policyCard}>
        <div class={style.cardTitle}>
          <h2>官方 ONNX</h2>
          <span>{store.state.policies.filter((policy) => policy.cached).length} 个已缓存</span>
        </div>
        {error.value && <p class={style.error}>{error.value}</p>}
        <div class={style.policyList}>
          {store.state.policies.map((policy) => (
            <article class={style.policyRow}>
              <div class={style.policyMain}>
                <strong>{policy.name}</strong>
                {policy.cached ? (
                  <>
                    <code>{policy.file}</code>
                    <small>{policy.size} bytes · 已缓存</small>
                  </>
                ) : (
                  <small>尚未从 Hugging Face 下载</small>
                )}
              </div>
              <div class={style.policyActions}>
                {policy.cached ? (
                  <span class={style.cachedLabel}>已缓存</span>
                ) : (
                  <PolicyDownloadButton policyId={policy.id} />
                )}
              </div>
            </article>
          ))}
        </div>
        <p class={style.muted}>
          来源：Hugging Face 的 microduck-policies，文件保存在应用缓存目录的
          library/policies/official。
        </p>
      </section>
    </div>
  );
});
