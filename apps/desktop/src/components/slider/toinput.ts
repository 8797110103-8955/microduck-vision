import { useEventListener } from "@vueuse/core";
import { LogicalPosition, getCurrentWindow } from "@tauri-apps/api/window";
import { onScopeDispose, toValue, type MaybeRefOrGetter } from "vue";
import type { SliderProps, SliderStepType } from "./interface";
import { decimalPlaces, formatText } from "./interface";
import style from "./style.module.scss";

export enum SliderValueStatus {
  Start = "start",
  Changing = "changing",
  End = "end",
  TextCommit = "textCommit",
}
export type SliderValueSource = "mouse" | "keyboard" | "blur";
export interface SliderEventInit {
  value: number;
  valueStatus: SliderValueStatus;
  valueSource: SliderValueSource;
  valueSourceButton?: number;
  inputType?: string;
}
type SliderDispatch = (event: SliderEventInit) => void;

export interface SliderInputSetup {
  track: MaybeRefOrGetter<HTMLElement | undefined>;
  input: MaybeRefOrGetter<HTMLInputElement | undefined>;
  props: SliderProps;
  valueNumber: MaybeRefOrGetter<number>;
  valueString: MaybeRefOrGetter<string>;
  dispatch: SliderDispatch;
}

type PointerScope = {
  startX: number;
  startValue: number;
  lastX: number;
  lastValue: number;
  moved: boolean;
};

let hiddenCursorUsers = 0;
let hiddenCursorPosition: LogicalPosition | undefined;
const hasTauriRuntime = "__TAURI_INTERNALS__" in globalThis || "__TAURI__" in globalThis;

function hideDraggingCursor(position: LogicalPosition) {
  if (hiddenCursorUsers++ !== 0) return;
  hiddenCursorPosition = position;
  document.documentElement.classList.add(style.dragging);
  if (hasTauriRuntime)
    void getCurrentWindow()
      .setCursorVisible(false)
      .catch(() => undefined);
}

function showDraggingCursor() {
  hiddenCursorUsers = Math.max(0, hiddenCursorUsers - 1);
  if (hiddenCursorUsers !== 0) return;
  document.documentElement.classList.remove(style.dragging);
  if (hasTauriRuntime) {
    void (async () => {
      try {
        const window = getCurrentWindow();
        if (hiddenCursorPosition) await window.setCursorPosition(hiddenCursorPosition);
        await window.setCursorVisible(true);
      } catch {
        // CSS cursor hiding remains the fallback outside an allowed Tauri runtime.
      }
    })();
  }
}

const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value));

function bounds(props: SliderProps, source: SliderValueSource) {
  return {
    min: source === "mouse" ? props.min : (props.minimum ?? props.min),
    max: source === "mouse" ? props.max : (props.maximum ?? props.max),
  };
}

function cycle(value: number, props: SliderProps) {
  if (!props.valueCycle) return value;
  const min = props.min;
  const max = props.max;
  const size = max - min;
  if (!size) return min;
  if (value > max) return min + ((value - min) % size);
  if (value < min) return max - ((min - value) % size);
  return value;
}

function dispatchValue(
  props: SliderProps,
  dispatch: SliderDispatch,
  value: number,
  valueSource: SliderValueSource,
  valueStatus: SliderValueStatus,
  valueSourceButton?: number,
): number | undefined {
  if (!Number.isFinite(value)) return undefined;
  const range = bounds(props, valueSource);
  let next = cycle(clamp(value, range.min, range.max), props);
  const step = Math.abs(props.step);
  if (step > 0) {
    next = Number(
      (range.min + Math.round((next - range.min) / step) * step).toFixed(decimalPlaces(step)),
    );
  }
  dispatch({
    value: next,
    valueSource,
    valueStatus,
    valueSourceButton,
    inputType: "insertText",
  });
  return next;
}

export function setupPointerInput(setup: SliderInputSetup) {
  let cleanupPointer: (() => void) | undefined;
  onScopeDispose(() => cleanupPointer?.());

  useEventListener(setup.track, "pointerdown", (event: PointerEvent) => {
    event.preventDefault();
    const track = toValue(setup.track);
    if (setup.props.disabled || !track) return;
    if (event.button === 2) {
      dispatchValue(
        setup.props,
        setup.dispatch,
        setup.props.default ?? toValue(setup.valueNumber),
        "mouse",
        SliderValueStatus.TextCommit,
        event.button,
      );
      return;
    }
    if (event.button !== 0) return;

    cleanupPointer?.();
    const drag: PointerScope = {
      startX: event.clientX,
      startValue: toValue(setup.valueNumber),
      lastX: event.clientX,
      lastValue: toValue(setup.valueNumber),
      moved: false,
    };
    hideDraggingCursor(new LogicalPosition(event.clientX, event.clientY));
    const onPointermove = (moveEvent: PointerEvent) => {
      const activeTrack = toValue(setup.track);
      if (!activeTrack) return;
      const rect = activeTrack.getBoundingClientRect();
      const wasMoved = drag.moved;
      drag.moved = true;
      const next = valueFromPointer(setup.props.stepType, setup.props, drag, moveEvent, rect);
      drag.lastX = moveEvent.clientX;
      drag.lastValue =
        dispatchValue(
          setup.props,
          setup.dispatch,
          next,
          "mouse",
          wasMoved ? SliderValueStatus.Changing : SliderValueStatus.Start,
          moveEvent.button,
        ) ?? drag.lastValue;
    };
    const onPointerup = (upEvent: PointerEvent) => {
      const input = toValue(setup.input);
      if (drag.moved)
        dispatchValue(
          setup.props,
          setup.dispatch,
          drag.lastValue,
          "mouse",
          SliderValueStatus.End,
          upEvent.button,
        );
      else input?.focus();
    };
    const finish = (finishEvent: PointerEvent) => {
      try {
        onPointerup(finishEvent);
      } finally {
        cleanupPointer?.();
      }
    };
    cleanupPointer = () => {
      document.removeEventListener("pointermove", onPointermove);
      document.removeEventListener("pointerup", finish, true);
      document.removeEventListener("pointercancel", finish, true);
      showDraggingCursor();
      cleanupPointer = undefined;
    };
    document.addEventListener("pointermove", onPointermove);
    document.addEventListener("pointerup", finish, { capture: true, once: true });
    document.addEventListener("pointercancel", finish, { capture: true, once: true });
  });

  useEventListener(setup.track, "contextmenu", (event: MouseEvent) => event.preventDefault());
}

function valueFromPointer(
  stepType: SliderStepType,
  props: SliderProps,
  drag: { startX: number; startValue: number; lastX: number; lastValue: number },
  event: PointerEvent,
  rect: DOMRect,
) {
  if (typeof stepType === "function") return stepType({ e: event, clientRect: rect });
  if (props.valueTick) {
    const movement = event.movementX || event.clientX - drag.lastX;
    const direct = Math.abs(movement) < 8 ? Math.sign(movement) : movement;
    let step = props.step * direct;
    if (event.shiftKey) step *= 10;
    if (event.ctrlKey) step /= 10;
    return drag.lastValue + step;
  }
  if (stepType === "movement") {
    const distance = event.movementX || event.clientX - drag.lastX;
    const direct = Math.abs(distance) < 8 ? Math.sign(distance) : distance;
    let step = props.step * direct;
    if (event.shiftKey) step *= 10;
    if (event.ctrlKey) step /= 10;
    return drag.lastValue + step;
  }
  if (stepType === "ratio")
    return drag.startValue + ((event.clientX - drag.startX) / rect.width) * (props.max - props.min);
  const distance = event.movementX || event.clientX - drag.lastX;
  const direct = distance > 0 ? 1 : distance < 0 ? -1 : 0;
  let step = props.step * direct;
  if (event.shiftKey) step *= 10;
  if (event.ctrlKey) step /= 10;
  return drag.lastValue + step;
}

export function setupKeyboardInput(setup: SliderInputSetup) {
  let hasInput = false;
  let lastValue = toValue(setup.valueNumber);
  let handlesKeyup = false;
  useEventListener(setup.input, "keydown", (event: KeyboardEvent) => {
    if (setup.props.disabled) return;
    if (event.key === "Enter") {
      event.preventDefault();
      toValue(setup.input)?.blur();
      return;
    }
    if (event.key !== "ArrowUp" && event.key !== "ArrowDown") return;
    event.preventDefault();
    let step = setup.props.step * (event.key === "ArrowUp" ? 1 : -1);
    if (event.shiftKey) step *= 10;
    if (!hasInput) lastValue = toValue(setup.valueNumber);
    lastValue =
      dispatchValue(
        setup.props,
        setup.dispatch,
        lastValue + step,
        "keyboard",
        hasInput ? SliderValueStatus.Changing : SliderValueStatus.Start,
      ) ?? lastValue;
    hasInput = true;
    handlesKeyup = true;
  });
  useEventListener(setup.input, "keyup", (event: KeyboardEvent) => {
    if (event.key === "Escape") {
      event.preventDefault();
      toValue(setup.input)?.blur();
      return;
    }
    if (!handlesKeyup || !hasInput) return;
    if (event.key !== "ArrowUp" && event.key !== "ArrowDown") return;
    event.preventDefault();
    hasInput = false;
    handlesKeyup = false;
    dispatchValue(setup.props, setup.dispatch, lastValue, "keyboard", SliderValueStatus.End);
  });
  useEventListener(
    setup.input,
    "blur",
    () => {
      hasInput = false;
      handlesKeyup = false;
      const input = toValue(setup.input);
      if (!input) return;
      if (input.value === "") {
        requestAnimationFrame(() => (input.value = toValue(setup.valueString)));
        return;
      }
      let valueNumber = Number(formatText(input.value));
      const currentValue = toValue(setup.valueNumber);
      if (currentValue === valueNumber) return;
      if (!Number.isFinite(valueNumber)) valueNumber = currentValue;
      setup.dispatch({
        value: valueNumber,
        valueSource: "blur",
        valueStatus: SliderValueStatus.TextCommit,
        inputType: "insertReplacementText",
      });
    },
    true,
  );
  useEventListener(
    setup.track,
    "input",
    (event) => {
      // @ts-expect-error
      if (!event[isSliderEvent]) event.stopImmediatePropagation();
    },
    true,
  );
}
export const isSliderEvent = Symbol("isSliderEvent");
