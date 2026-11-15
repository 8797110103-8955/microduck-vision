import type { ExtractPropTypes, PropType } from "vue";

export type SliderStepType =
  | "step"
  | "movement"
  | "ratio"
  | ((event: { e: PointerEvent; clientRect: DOMRect }) => number);

export const sliderProps = {
  modelValue: { type: [String, Number] },
  value: { type: [String, Number] },
  min: { type: Number, default: -Number.MAX_VALUE },
  max: { type: Number, default: Number.MAX_VALUE },
  minimum: { type: Number },
  maximum: { type: Number },
  step: { type: Number, default: 1 },
  stepDisplay: { type: Number },
  stepValid: { type: Number, default: 0.0000004 },
  stepType: {
    // oxlint-disable-next-line no-unsafe-type-assertion
    type: [String, Function] as PropType<SliderStepType>,
    default: "movement" as const,
  },
  default: { type: Number },
  valueAttach: { type: String, default: "" },
  valueHidden: { type: Boolean, default: false },
  valueFormater: {
    // oxlint-disable-next-line no-unsafe-type-assertion
    type: Function as PropType<(value: number) => string | number>,
  },
  valueCycle: { type: Boolean, default: false },
  valueTick: { type: Boolean, default: false },
  progress: { type: [Boolean, String] as PropType<boolean | string> },
  progressCenter: { type: [Boolean, Number, String] as PropType<boolean | number | string> },
  suffix: { type: String, default: "" },
  showValue: { type: Boolean, default: true },
  disabled: { type: Boolean, default: false },
  ariaLabel: { type: String, default: "" },
};

export type SliderProps = ExtractPropTypes<typeof sliderProps>;

export function decimalPlaces(value: number) {
  const text = String(value);
  const index = text.indexOf(".");
  return index < 0 ? 0 : text.length - index - 1;
}

export function formatStep(value: number, step: number) {
  return Number(value.toFixed(decimalPlaces(step)));
}

export function formatText(value: string) {
  let next = value.replace(/[^\d.-]/g, "");
  const negative = next.startsWith("-");
  next = next.replace(/-/g, "");
  if (next.startsWith(".")) next = `0${next}`;
  let dots = 0;
  next = next.replace(/\./g, (match) => (dots++ ? "" : match));
  return negative ? `-${next}` : next;
}
