import { defineComponent, onScopeDispose, ref, shallowRef, watch } from "vue";
import { useEventListener } from "@vueuse/core";
import type { SceneFrame, SceneInfo, SimulationState, WorldState } from "../../../generated/types";
import {
  defaultPoseEvent,
  focusDuckEvent,
  requestServoSelection,
  restoreDuckViewEvent,
  servoSelectionEvent,
} from "../viewer-events";
import { DuckViewer } from "./viewer";
import style from "./style.module.scss";

/**
 * Right edge of the official jaw.stl mesh at the mouth seam for the default
 * camera after its MJCF geom transform, expressed in the jaw_soft body frame.
 * The view's horizontal axis combines model x/y, so this is intentionally the
 * right-side x/y point with the lower seam z value.
 */
const mouthMarkerRightEdge: [number, number, number] = [0.0119914336, 0.04442375898, -0.0751049415];
const markerLabel = {
  online: "在线",
  warn: "告警",
  missing: "缺失",
  unknown: "未知",
};

export type DisplayFrame =
  | { kind: "kinematic"; value: SceneFrame }
  | {
      kind: "simulation";
      value: Pick<SimulationState, "frame" | "trunkPosition" | "trunkQuaternion">;
      preserveWorld?: boolean;
    };

type StageView = {
  scene: SceneInfo | null;
  frame?: DisplayFrame;
  world?: WorldState;
  worldPositions: [number, number, number][];
  worldPoses: (WorldState | undefined)[];
  selected: number;
  labels: { id: string; name: string }[];
  markers: {
    name: string;
    body: number;
    id: number;
    physicalId: number | null;
    appearance: keyof typeof markerLabel;
  }[];
  highlightBodies: number[];
  debug: boolean;
  tabs: { label: string; active: boolean; select: () => void }[];
};

export const OfficialDuckStage = defineComponent(
  (props: {
    view: StageView;
    onReady: () => void;
    onDefaultPose: () => void;
    onSelectDuck: (index: number) => void;
  }) => {
    const host = shallowRef<HTMLDivElement>();
    let viewer: DuckViewer | undefined;
    const markerPos = ref<ReturnType<DuckViewer["markerScreen"]>[]>([]);
    const labelPos = ref<ReturnType<DuckViewer["duckScreen"]>[]>([]);
    const selectedServoId = ref<number | null>(null);

    function placeDucks() {
      const { worldPositions, worldPoses, selected } = props.view;
      viewer?.setGhosts(Math.max(worldPositions.length - 1, 0), 0.25, worldPositions);
      viewer?.setMachineWorldPoses(worldPoses);
      viewer?.setSelectedDuck(selected);
    }

    function placeActiveDuck() {
      const world = props.view.world;
      viewer?.setWorldPose(world?.position ?? [0, 0], world?.yaw ?? 0);
    }

    function applyFrame() {
      const frame = props.view.frame;
      if (frame?.kind === "simulation") {
        placeActiveDuck();
        viewer?.setSimulationFrame(frame.value, frame.preserveWorld);
      } else if (frame) {
        viewer?.setFrame(frame.value);
      }
    }

    watch([() => host.value, () => props.view.scene], ([element, scene]) => {
      if (!element || !scene) return;
      viewer?.dispose();
      const instance = new DuckViewer(element, scene);
      viewer = instance;
      instance.onRender = () => {
        const { debug, markers, highlightBodies, labels } = props.view;
        if (debug) {
          markerPos.value = markers.map((marker, index) =>
            index === 9
              ? instance.markerScreenAt(marker.body, mouthMarkerRightEdge)
              : instance.markerScreen(index),
          );
        }
        instance.highlightBodies(highlightBodies);
        labelPos.value = labels.map((_, index) => instance.duckScreen(index));
      };
      placeDucks();
      placeActiveDuck();
      applyFrame();
      props.onReady();
    });
    watch(() => props.view.frame, applyFrame);
    watch(() => props.view.world, placeActiveDuck, { deep: true });
    watch(
      [
        () => props.view.worldPositions,
        () => props.view.worldPoses,
        () => props.view.selected,
        () => props.view.debug,
      ],
      placeDucks,
      { deep: true },
    );

    useEventListener(window, focusDuckEvent, () => viewer?.focusCurrentDuck());
    useEventListener(window, restoreDuckViewEvent, () => viewer?.restorePreviousView());
    useEventListener(window, defaultPoseEvent, () => {
      if (!props.view.scene) return;
      props.onDefaultPose();
      viewer?.setFrame(props.view.scene.defaultFrame);
    });
    useEventListener(window, servoSelectionEvent, (event: Event) => {
      if (event instanceof CustomEvent) {
        selectedServoId.value = typeof event.detail === "number" ? event.detail : null;
      }
    });
    onScopeDispose(() => viewer?.dispose());

    return () => (
      <section class={style.officialDuckStage}>
        <div class={style.stageBar}>
          <div class={style.seg}>
            {props.view.tabs.map((tab) => (
              <button
                key={tab.label}
                class={tab.active ? style.on : undefined}
                onClick={tab.select}
              >
                {tab.label}
              </button>
            ))}
          </div>
        </div>
        <div class={style.stageBody}>
          <div
            data-testid="duck-viewport"
            class={style.viewport}
            ref={host}
            onClick={(event: MouseEvent) => {
              const index = viewer?.pickDuck(event.clientX, event.clientY);
              if (index != null) props.onSelectDuck(index);
            }}
          >
            {props.view.labels.map((label, index) => {
              const position = labelPos.value[index];
              if (!position) return null;
              return (
                <span
                  key={label.id}
                  class={style.duckLabel}
                  style={{ left: `${position.x}px`, top: `${position.y}px` }}
                >
                  {label.name}
                </span>
              );
            })}
            {props.view.debug &&
              props.view.markers.map((marker, index) => {
                const position = markerPos.value[index];
                if (!position) return null;
                return (
                  <button
                    key={marker.name}
                    class={[
                      style.marker,
                      style[marker.appearance],
                      selectedServoId.value === marker.id ? style.active : undefined,
                    ]}
                    style={{ left: `${position.x}px`, top: `${position.y}px` }}
                    title={`${marker.name} · ${marker.physicalId == null ? "未匹配" : `ID ${marker.physicalId}`} · ${markerLabel[marker.appearance]}`}
                    onClick={(event: MouseEvent) => {
                      event.stopPropagation();
                      requestServoSelection(selectedServoId.value === marker.id ? null : marker.id);
                    }}
                  >
                    {marker.physicalId ?? "—"}
                  </button>
                );
              })}
          </div>
        </div>
      </section>
    );
  },
  { props: ["view", "onReady", "onDefaultPose", "onSelectDuck"] },
);
