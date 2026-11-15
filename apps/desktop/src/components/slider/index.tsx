import { computed, defineComponent, ref, shallowRef, toValue } from "vue";
import { formatStep, sliderProps } from "./interface";
import { useEventListener } from "@vueuse/core";
import {
  setupKeyboardInput,
  setupPointerInput,
  SliderValueStatus,
  type SliderValueSource,
  type SliderEventInit,
  isSliderEvent,
} from "./toinput";
import style from "./style.module.scss";

const DEFAULT_PROGRESS = "#f5b700";

export { SliderValueStatus } from "./toinput";
export class SliderEvent extends InputEvent {
  private readonly [isSliderEvent] = true;
  value: number;
  valueStatus: SliderValueStatus;
  valueSource: SliderValueSource;
  valueSourceButton?: number;
  constructor(
    init = {
      value: 0,
      valueStatus: SliderValueStatus.TextCommit,
      valueSource: "keyboard",
    } as SliderEventInit & InputEventInit,
  ) {
    super("input", init);
    this.value = init.value;
    this.valueStatus = init.valueStatus;
    this.valueSource = init.valueSource;
    this.valueSourceButton = init.valueSourceButton;
  }
}

export const Slider = defineComponent({
  name: "Slider",
  inheritAttrs: false,
  props: sliderProps,
  emits: ["update:modelValue", "input"],
  setup(props, { attrs, emit }) {
    const track = shallowRef<HTMLDivElement>();
    const input = shallowRef<HTMLInputElement>();
    const focused = ref(false);

    const formatter = (value: number) => props.valueFormater?.(value) ?? value;
    const valueNumber = computed(() => {
      const value = props.modelValue ?? props.value ?? props.default ?? props.min;
      const numeric = Number(value);
      return Number.isFinite(numeric) ? numeric : 0;
    });
    const valueString = computed(() => {
      if (props.valueHidden && !focused.value) return "";
      const value = formatStep(valueNumber.value, props.stepDisplay ?? props.step);
      const rendered = formatter(value);
      return `${rendered}${props.valueAttach || props.suffix}`;
    });

    const setup = {
      track,
      input,
      props,
      valueNumber,
      valueString,
      dispatch: (event: SliderEventInit) => {
        emit("update:modelValue", event.value);

        const e = new SliderEvent(event);
        emit("input", e);
        const ele = toValue(track)!;
        // @ts-expect-error;
        ele.value = event.value;
        ele.dispatchEvent(e);
      },
    };
    setupPointerInput(setup);
    setupKeyboardInput(setup);

    useEventListener(setup.input, "focus", () => {
      focused.value = true;
      requestAnimationFrame(() => toValue(setup.input)?.select());
    });
    useEventListener(setup.input, "blur", () => {
      focused.value = false;
    });
    const progressStyle = computed(() => {
      if (!props.progress || focused.value) return undefined;
      const { min, max } = props;
      const value = Math.min(max, Math.max(min, valueNumber.value));
      const color = props.progress === true ? DEFAULT_PROGRESS : props.progress;
      const center = progressCenter(props.progressCenter, min, max);
      if (center == null) {
        const rate = ((value - min) / Math.max(max - min, Number.EPSILON)) * 100;
        return {
          backgroundImage: `linear-gradient(to right, ${color} 0 ${rate}%, transparent ${rate}%)`,
        };
      }
      const centerRate = ((center - min) / Math.max(max - min, Number.EPSILON)) * 100;
      if (value >= center) {
        const rate =
          ((value - center) / Math.max(max - center, Number.EPSILON)) * (100 - centerRate);
        return {
          backgroundImage: `linear-gradient(to right, transparent 0 ${centerRate}%, ${color} ${centerRate}% ${centerRate + rate}%, transparent ${centerRate + rate}%)`,
        };
      }
      const rate = ((center - value) / Math.max(center - min, Number.EPSILON)) * centerRate;
      return {
        backgroundImage: `linear-gradient(to right, transparent 0 ${centerRate - rate}%, ${color} ${centerRate - rate}% ${centerRate}%, transparent ${centerRate}%)`,
      };
    });
    return () => (
      <div
        ref={track}
        {...attrs}
        class={[style.track, props.disabled && style.disabled, attrs.class]}
        style={[progressStyle.value, attrs.style]}
        data-slider="true"
      >
        <div class={style.overlay} />
        <input
          ref={input}
          class={[style.input, !props.showValue && style.hiddenInput]}
          type="text"
          value={props.showValue ? valueString.value : ""}
          disabled={props.disabled}
          aria-label={props.showValue ? props.ariaLabel || undefined : undefined}
          tabindex={props.showValue ? 0 : -1}
        />
        {props.showValue && props.suffix && <span class={style.suffix}>{props.suffix}</span>}
      </div>
    );
  },
});

function progressCenter(value: boolean | number | string | undefined, min: number, max: number) {
  if (value === undefined || value === false) return null;
  if (value === true) return (min + max) / 2;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : (min + max) / 2;
}
