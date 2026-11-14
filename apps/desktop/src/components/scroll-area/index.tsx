import { defineComponent, onMounted, onUnmounted, ref } from "vue";
import style from "./style.module.scss";

export const ScrollArea = defineComponent({
  name: "ScrollArea",
  props: {
    className: { type: String, default: "" },
    fill: { type: Boolean, default: false },
  },
  setup(props, { slots }) {
    const viewport = ref<HTMLDivElement>();
    const rail = ref<HTMLDivElement>();
    const thumbSize = ref(100);
    const thumbOffset = ref(0);
    const scrollable = ref(false);
    let resizeObserver: ResizeObserver | null = null;
    let dragging: { pointerId: number; startY: number; startScrollTop: number } | null = null;

    const sync = () => {
      const element = viewport.value;
      if (!element) return;
      const maxScrollTop = element.scrollHeight - element.clientHeight;
      scrollable.value = maxScrollTop > 1;
      if (!scrollable.value) {
        thumbSize.value = 100;
        thumbOffset.value = 0;
        return;
      }
      const size = Math.max(12, (element.clientHeight / element.scrollHeight) * 100);
      thumbSize.value = Math.min(size, 100);
      thumbOffset.value = (element.scrollTop / maxScrollTop) * (100 - thumbSize.value);
    };

    const scrollToRailPosition = (clientY: number) => {
      const element = viewport.value;
      const track = rail.value;
      if (!element || !track) return;
      const bounds = track.getBoundingClientRect();
      const ratio = Math.min(1, Math.max(0, (clientY - bounds.top) / bounds.height));
      element.scrollTop = ratio * (element.scrollHeight - element.clientHeight);
    };

    const onThumbPointerDown = (event: PointerEvent) => {
      if (!scrollable.value || !viewport.value) return;
      event.preventDefault();
      event.stopPropagation();
      dragging = {
        pointerId: event.pointerId,
        startY: event.clientY,
        startScrollTop: viewport.value.scrollTop,
      };
      if (event.currentTarget instanceof HTMLElement)
        event.currentTarget.setPointerCapture(event.pointerId);
    };

    const onThumbPointerMove = (event: PointerEvent) => {
      if (!dragging || dragging.pointerId !== event.pointerId || !viewport.value || !rail.value)
        return;
      const trackHeight = rail.value.clientHeight;
      const thumbHeight = (thumbSize.value / 100) * trackHeight;
      const scrollRange = viewport.value.scrollHeight - viewport.value.clientHeight;
      const thumbRange = Math.max(1, trackHeight - thumbHeight);
      viewport.value.scrollTop =
        dragging.startScrollTop + ((event.clientY - dragging.startY) / thumbRange) * scrollRange;
    };

    const onThumbPointerUp = (event: PointerEvent) => {
      if (dragging?.pointerId === event.pointerId) dragging = null;
    };

    const onRailPointerDown = (event: PointerEvent) => {
      if (event.target === rail.value) scrollToRailPosition(event.clientY);
    };

    const onThumbKeydown = (event: KeyboardEvent) => {
      const element = viewport.value;
      if (!element) return;
      if (event.key === "ArrowDown") element.scrollTop += 40;
      if (event.key === "ArrowUp") element.scrollTop -= 40;
      if (event.key === "PageDown") element.scrollTop += element.clientHeight;
      if (event.key === "PageUp") element.scrollTop -= element.clientHeight;
      if (event.key === "Home") element.scrollTop = 0;
      if (event.key === "End") element.scrollTop = element.scrollHeight;
      if (["ArrowDown", "ArrowUp", "PageDown", "PageUp", "Home", "End"].includes(event.key)) {
        event.preventDefault();
      }
    };

    onMounted(() => {
      const element = viewport.value;
      if (!element) return;
      element.addEventListener("scroll", sync, { passive: true });
      window.addEventListener("resize", sync);
      if (typeof ResizeObserver !== "undefined") {
        resizeObserver = new ResizeObserver(sync);
        resizeObserver.observe(element);
      }
      sync();
    });

    onUnmounted(() => {
      viewport.value?.removeEventListener("scroll", sync);
      window.removeEventListener("resize", sync);
      resizeObserver?.disconnect();
      resizeObserver = null;
    });

    return () => (
      <div class={[style.scrollArea, props.fill && style.fill, props.className]}>
        <div class={style.viewport} ref={viewport}>
          {slots.default?.()}
        </div>
        <div
          class={[style.rail, scrollable.value && style.visible]}
          ref={rail}
          onPointerdown={onRailPointerDown}
          aria-hidden={!scrollable.value}
        >
          {scrollable.value && (
            <div
              class={style.thumb}
              role="scrollbar"
              tabindex="0"
              aria-valuemin="0"
              aria-valuemax="100"
              aria-valuenow={Math.round(
                (thumbOffset.value / Math.max(1, 100 - thumbSize.value)) * 100,
              )}
              style={{ height: `${thumbSize.value}%`, top: `${thumbOffset.value}%` }}
              onPointerdown={onThumbPointerDown}
              onPointermove={onThumbPointerMove}
              onPointerup={onThumbPointerUp}
              onPointercancel={onThumbPointerUp}
              onKeydown={onThumbKeydown}
            />
          )}
        </div>
      </div>
    );
  },
});
