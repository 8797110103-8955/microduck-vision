export const focusDuckEvent = "microduck:focus-current-duck";
export const restoreDuckViewEvent = "microduck:restore-duck-view";
export const servoSelectionEvent = "microduck:select-servo";
export const defaultPoseEvent = "microduck:default-pose";

export function requestDuckFocus() {
  window.dispatchEvent(new Event(focusDuckEvent));
}

export function requestRestoreDuckView() {
  window.dispatchEvent(new Event(restoreDuckViewEvent));
}

export function requestServoSelection(id: number | null) {
  window.dispatchEvent(new CustomEvent<number | null>(servoSelectionEvent, { detail: id }));
}

export function requestDefaultPose() {
  window.dispatchEvent(new Event(defaultPoseEvent));
}
