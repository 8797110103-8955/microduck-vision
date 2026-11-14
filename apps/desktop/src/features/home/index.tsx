import { computed, defineComponent, onMounted, onUnmounted, ref, shallowRef, watch } from "vue";
import { ScrollArea } from "../../components/scroll-area";
import { desktopBackend, useDesktopStore } from "../../store";
import {
  DebugPage,
  debugPhysicalMachineFor,
  load,
  officialSceneFrame,
  officialSceneModel,
  sendAction,
} from "../debug";
import { DuckViewer } from "./viewer";
import style from "./style.module.scss";

const JOINT_COUNT = 15;
const markerLabel = {
  online: "在线",
  warn: "告警",
  missing: "缺失",
  unknown: "未知",
};

export const OfficialDuckStage = defineComponent(() => {
  const store = useDesktopStore();
  const host = ref<HTMLDivElement>();
  const viewer = shallowRef<DuckViewer>();
  const scene = ref<Awaited<ReturnType<typeof officialSceneModel>> | null>(null);
  const frame = ref<Awaited<ReturnType<typeof officialSceneFrame>> | null>(null);
  const debug = computed(() => store.state.tab === "debug");
  const activeLoop = computed(() => (debug.value ? store.state.loop : store.state.previewLoop));
  const markerPos = ref<({ x: number; y: number } | null)[]>([]);
  const labelPos = ref<({ x: number; y: number } | null)[]>([]);

  const mappedMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  /** Every virtual machine in the scene; the selected one is the live duck, the rest stand beside it. */
  const sceneMachines = computed(() => {
    const machines = store.state.virtualMachines;
    const selected = machines.find((machine) => machine.id === store.state.selectedVirtualMachine);
    return selected ? [selected, ...machines.filter((machine) => machine !== selected)] : machines;
  });
  const ghostCount = () => (debug.value ? 0 : Math.max(sceneMachines.value.length - 1, 0));
  const debugName = () => selectedMachine.value?.name ?? "官方小鸭子";
  /** Names above the eyes: every machine in preview, only the one being debugged in debug. */
  const labels = computed(() =>
    debug.value
      ? [{ id: selectedMachine.value?.id ?? "official", name: debugName() }]
      : sceneMachines.value,
  );
  const markers = computed(() =>
    (scene.value?.model.joints ?? []).map((joint, index) => ({
      ...joint,
      index,
      id: index + 1,
    })),
  );
  const markerState = (id: number) => {
    const online = mappedMachine.value?.onlineServoIds ?? [];
    if (!mappedMachine.value) return "unknown";
    if (activeLoop.value?.sensorFailures) return "warn";
    return online.includes(id) ? "online" : "missing";
  };

  async function refreshFrame(positions: number[]) {
    if (!scene.value) return;
    frame.value = await officialSceneFrame(
      positions.length === JOINT_COUNT ? positions : Array(JOINT_COUNT).fill(0),
    );
    viewer.value?.setFrame(frame.value);
  }

  async function buildViewer() {
    if (!host.value || !scene.value) return;
    viewer.value?.dispose();
    viewer.value = new DuckViewer(host.value, scene.value);
    viewer.value.onRender = () => {
      const v = viewer.value;
      if (!v) return;
      if (debug.value) markerPos.value = markers.value.map((_, index) => v.markerScreen(index));
      labelPos.value = labels.value.map((_, index) => v.duckScreen(index));
    };
    viewer.value.setGhosts(ghostCount());
    await refreshFrame(activeLoop.value?.positions ?? Array(JOINT_COUNT).fill(0));
  }

  onMounted(async () => {
    scene.value = await officialSceneModel();
    await buildViewer();
  });

  onUnmounted(() => {
    viewer.value?.dispose();
  });

  watch(
    () => activeLoop.value?.positions,
    (positions) => {
      if (positions) void refreshFrame(positions);
    },
  );
  watch([() => debug.value, () => sceneMachines.value.length], () =>
    viewer.value?.setGhosts(ghostCount()),
  );

  return () => (
    <section class={style.officialDuckStage}>
      <div class={style.stageBar}>
        <div class={style.seg}>
          <button
            class={store.state.tab === "preview" ? style.on : undefined}
            onClick={() => (store.state.tab = store.state.tab === "preview" ? "home" : "preview")}
          >
            预览
          </button>
          <button
            class={debug.value ? style.on : undefined}
            onClick={() => (store.state.tab = debug.value ? "home" : "debug")}
          >
            调试
          </button>
        </div>
      </div>
      <div class={style.stageBody}>
        <div class={style.viewport} ref={host}>
          {labels.value.map((label, index) => {
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
          {debug.value &&
            markers.value.map((marker, index) => {
              const position = markerPos.value[index];
              if (!position) return null;
              const state = markerState(marker.id);
              return (
                <button
                  key={marker.name}
                  class={[style.marker, style[state]]}
                  style={{ left: `${position.x}px`, top: `${position.y}px` }}
                  title={`${marker.name} · ID ${marker.id} · ${markerLabel[state]}`}
                >
                  {marker.id}
                </button>
              );
            })}
        </div>
      </div>
    </section>
  );
});

export const PreviewPage = defineComponent(() => {
  const store = useDesktopStore();
  const busy = ref<string | null>(null);
  const error = ref<string | null>(null);
  const loopBusy = ref(false);
  let timer: number | undefined;
  let ticking = false;
  const selectedMachine = computed(() =>
    store.state.virtualMachines.find(
      (machine) => machine.id === store.state.selectedVirtualMachine,
    ),
  );
  const physicalMachine = computed(() =>
    debugPhysicalMachineFor(
      store.state.selectedVirtualMachine,
      store.state.virtualMachines,
      store.state.physicalMachines,
    ),
  );
  const cachedPolicies = computed(() => store.state.policies.filter((policy) => policy.cached));

  const refresh = async () => {
    try {
      store.state.policies = await desktopBackend().policyList();
      store.state.selectedPolicyId =
        store.state.policies.find((policy) => policy.selected)?.id ?? null;
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    }
  };

  const selectPolicy = async (id: string) => {
    busy.value = id;
    try {
      const selected = await desktopBackend().policySelect(id);
      store.state.selectedPolicyId = selected.id;
      if (store.state.previewLoop) {
        store.state.previewLoop = await desktopBackend().previewLoopPolicy(id);
      }
      await refresh();
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      busy.value = null;
    }
  };

  const startLoop = async () => {
    if (!selectedMachine.value || loopBusy.value) return;
    loopBusy.value = true;
    try {
      store.state.previewLoop = await desktopBackend().previewLoopOpen(selectedMachine.value.id);
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const stopLoop = async () => {
    if (loopBusy.value) return;
    loopBusy.value = true;
    try {
      await desktopBackend().previewLoopClose();
      store.state.previewLoop = null;
      error.value = null;
    } catch (reason) {
      error.value = reason instanceof Error ? reason.message : String(reason);
    } finally {
      loopBusy.value = false;
    }
  };

  const toggleLoop = () => {
    if (store.state.previewLoop) {
      void stopLoop();
      return;
    }
    void startLoop();
  };

  const stopTimer = () => {
    if (timer !== undefined) window.clearInterval(timer);
    timer = undefined;
  };

  const startTimer = () => {
    stopTimer();
    timer = window.setInterval(() => {
      if (!store.state.previewLoop || ticking) return;
      ticking = true;
      void desktopBackend()
        .previewLoopTick()
        .then((status) => {
          store.state.previewLoop = status;
        })
        .catch((reason) => {
          error.value = reason instanceof Error ? reason.message : String(reason);
        })
        .finally(() => {
          ticking = false;
        });
    }, 20);
  };

  onMounted(() => {
    if (!store.state.policies.length && !store.state.loading) void refresh();
    startTimer();
  });

  onUnmounted(stopTimer);

  return () => (
    <ScrollArea className={[style.page, style.previewPage].join(" ")}>
      <div class={style.previewMachineRow}>
        <strong>{selectedMachine.value?.name ?? "机器"}</strong>
        <span>{physicalMachine.value?.name ?? "未关联实机"}</span>
        <i
          class={[
            style.previewStatusDot,
            physicalMachine.value?.connected ? style.online : style.offline,
          ]}
          title={physicalMachine.value?.connected ? "实机在线" : "实机离线"}
        />
      </div>
      <div class={style.previewLoopBar}>
        <button type="button" disabled={loopBusy.value} onClick={toggleLoop}>
          {store.state.previewLoop ? "停止控制环" : "开启控制环"}
        </button>
        {store.state.previewLoop && (
          <small>{store.state.previewLoop.torque ? "策略运行中" : "控制环已开启，等待策略"}</small>
        )}
      </div>
      {error.value && <p class={style.previewError}>{error.value}</p>}
      <div class={style.previewPolicyList} aria-label="已缓存 ONNX 策略">
        {cachedPolicies.value.map((policy) => (
          <button
            class={store.state.selectedPolicyId === policy.id ? style.selected : undefined}
            type="button"
            disabled={busy.value !== null}
            onClick={() => void selectPolicy(policy.id)}
          >
            {policy.name}
          </button>
        ))}
        {!cachedPolicies.value.length && (
          <p class={style.previewEmpty}>当前电脑还没有已缓存的 ONNX</p>
        )}
      </div>
    </ScrollArea>
  );
});

export const HomePage = defineComponent(() => {
  const store = useDesktopStore();
  onMounted(() => {
    if (!store.state.catalog.length && !store.state.loading) void load();
  });

  return () => (
    <div
      class={[style.page, style.homePage, store.state.tab === "settings" && style.hidden]}
      aria-hidden={store.state.tab === "settings"}
    >
      <OfficialDuckStage />
      {store.state.tab === "debug" && <DebugPage />}
      {store.state.tab === "preview" && <PreviewPage />}
      <button
        class={[style.danger, style.homeEstop]}
        disabled={!store.state.loop}
        onClick={() => void sendAction({ kind: "estop" })}
      >
        急停 <small>Esc</small>
      </button>
    </div>
  );
});
