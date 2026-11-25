import { defineComponent, shallowRef, useAttrs } from "vue";
import style from "./style.module.scss";

/** Native select with one shared shell and option content supplied by the caller. */
export const Select = defineComponent({
  name: "Select",
  inheritAttrs: false,
  props: {
    value: { type: [String, Number], default: undefined },
    disabled: { type: Boolean, default: false },
    onChange: Function,
    onClick: Function,
  },
  emits: {
    selectionChange: (_select: HTMLSelectElement) => true,
  },
  setup(props, { emit, slots }) {
    const attrs = useAttrs();
    const select = shallowRef<HTMLSelectElement>();
    return () => {
      const { class: className, ...selectAttrs } = attrs;
      return (
        <span class={[style.select, className]}>
          <select
            {...selectAttrs}
            ref={select}
            value={props.value}
            disabled={props.disabled}
            onChange={(event: Event) => {
              if (select.value) emit("selectionChange", select.value);
              if (typeof props.onChange === "function") props.onChange(event);
            }}
            onClick={(event: PointerEvent) => {
              if (typeof props.onClick === "function") props.onClick(event);
            }}
          >
            {slots.default?.()}
          </select>
          <span class={style.chevron} aria-hidden="true" />
        </span>
      );
    };
  },
});
