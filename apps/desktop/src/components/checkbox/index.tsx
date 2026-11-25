import { defineComponent, shallowRef } from "vue";
import style from "./style.module.scss";

export const Checkbox = defineComponent({
  name: "Checkbox",
  props: {
    label: { type: String, required: true },
    checked: Boolean,
    disabled: Boolean,
    stacked: Boolean,
  },
  emits: {
    change: (_checked: boolean) => true,
  },
  setup(props, { emit }) {
    const input = shallowRef<HTMLInputElement>();
    return () => (
      <label class={[style.checkbox, props.stacked && style.stacked]}>
        <span>{props.label}</span>
        <span class={style.control}>
          <input
            ref={input}
            type="checkbox"
            checked={props.checked}
            disabled={props.disabled}
            onChange={() => {
              if (input.value) emit("change", input.value.checked);
            }}
          />
        </span>
      </label>
    );
  },
});
