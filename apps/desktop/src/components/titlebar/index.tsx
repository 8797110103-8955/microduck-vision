import { defineComponent, onMounted, onUnmounted, ref, shallowRef } from "vue";
import { useEventListener } from "@vueuse/core";
import { getCurrentWindow } from "@tauri-apps/api/window";
import style from "./style.module.scss";

export const Titlebar = defineComponent({
  name: "Titlebar",
  props: {
    min: { type: Boolean, default: true },
    max: { type: Boolean, default: true },
    close: { type: Boolean, default: true },
  },
  setup(props, { slots }) {
    const maximized = ref(false);
    const minButton = shallowRef<HTMLButtonElement>();
    const maxButton = shallowRef<HTMLButtonElement>();
    const closeButton = shallowRef<HTMLButtonElement>();
    let unlistenResize: (() => void) | null = null;

    const refreshMaximized = async () => {
      try {
        maximized.value = await getCurrentWindow().isMaximized();
      } catch {
        maximized.value = false;
      }
    };

    onMounted(() => {
      let appWindow: ReturnType<typeof getCurrentWindow>;
      try {
        appWindow = getCurrentWindow();
      } catch {
        return;
      }
      void (async () => {
        await refreshMaximized();
        try {
          unlistenResize = await appWindow.onResized(() => void refreshMaximized());
        } catch {
          unlistenResize = null;
        }
      })();
    });

    onUnmounted(() => {
      unlistenResize?.();
      unlistenResize = null;
    });

    useEventListener(minButton, "click", (event: MouseEvent) => {
      event.stopPropagation();
      void getCurrentWindow().minimize();
    });
    useEventListener(maxButton, "click", (event: MouseEvent) => {
      event.stopPropagation();
      void (maximized.value ? getCurrentWindow().unmaximize() : getCurrentWindow().maximize());
    });
    useEventListener(closeButton, "click", (event: MouseEvent) => {
      event.stopPropagation();
      void getCurrentWindow().close();
    });

    return () => (
      <header class={style.titlebar}>
        <div class={style.leading} data-tauri-drag-region="deep">
          {slots.default?.()}
        </div>
        <div class={style.drag} data-tauri-drag-region="deep" />
        <div class={style.actions}>
          {slots.actions?.()}
          {props.min && (
            <button ref={minButton} class={style.control} aria-label="最小化" title="最小化">
              <span class={style.controlIcon} aria-hidden="true">
                {"\uE921"}
              </span>
            </button>
          )}
          {props.max && (
            <button
              ref={maxButton}
              id="snap-btn"
              class={style.control}
              aria-label="最大化或还原"
              title="最大化或还原"
            >
              <span class={style.controlIcon} aria-hidden="true">
                {maximized.value ? "\uE923" : "\uE922"}
              </span>
            </button>
          )}
          {props.close && (
            <button
              ref={closeButton}
              class={[style.control, style.close]}
              aria-label="关闭"
              title="关闭"
            >
              <span class={style.controlIcon} aria-hidden="true">
                {"\uE8BB"}
              </span>
            </button>
          )}
        </div>
      </header>
    );
  },
});
