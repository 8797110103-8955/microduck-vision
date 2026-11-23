import * as commands from "../../../generated/types";
import { computed, defineComponent, onScopeDispose, ref, shallowRef } from "vue";
import { useEventListener } from "@vueuse/core";
import { listen } from "@tauri-apps/api/event";
import type { LoopStatus, TrainingFrame, TrainingProgress } from "../../../generated/types";
import { getCurrentWindow } from "@tauri-apps/api/window";
import appIconUrl from "../../../assets/icons/icon.ico";
import { Message } from "../../components/message";
import { Titlebar } from "../../components/titlebar";
import { useDesktopStore } from "../../store";
import { load } from "../debug";
import { HomePage } from "../home";
import { refreshPhysicalMachines, refreshPhysicalPorts } from "../machines";
import { SettingsPage } from "../settings";
import style from "./style.module.scss";

const DesktopTitlebar = defineComponent(() => {
  const store = useDesktopStore();
  const servoDragOccurred = ref(false);
  const servoStatusButton = shallowRef<HTMLElement>();
  const appIcon = shallowRef<HTMLElement>();
  const settingsButton = shallowRef<HTMLButtonElement>();
  const machineTabButtons = new Map<string, ReturnType<typeof shallowRef<HTMLButtonElement>>>();
  for (const view of ["simulation", "servo", "physical"] as const) {
    const button = shallowRef<HTMLButtonElement>();
    machineTabButtons.set(view, button);
    useEventListener(button, "click", (event: MouseEvent) => {
      event.stopPropagation();
      store.state.machineView = view;
    });
  }
  let servoPointer: { id: number; x: number; y: number; active: boolean } | null = null;
  const onlineServos = computed(() =>
    store.state.physicalMachines.reduce(
      (total, machine) => total + machine.onlineServoIds.length,
      0,
    ),
  );
  const onlineMachines = computed(
    () => store.state.physicalMachines.filter((machine) => machine.connected).length,
  );

  const openServoStatus = (event: MouseEvent) => {
    if (servoDragOccurred.value) {
      event.preventDefault();
      event.stopPropagation();
      servoDragOccurred.value = false;
      return;
    }
    const mapped = store.state.physicalMachines.find((machine) => machine.virtualMachineId);
    if (onlineServos.value && mapped?.virtualMachineId) {
      store.state.selectedVirtualMachine = mapped.virtualMachineId;
      store.state.selectedVirtualMachineManual = true;
    }
    store.state.tab = "home";
  };

  const onServoPointerDown = (event: PointerEvent) => {
    if (event.button !== 0) return;
    servoDragOccurred.value = false;
    servoPointer = { id: event.pointerId, x: event.clientX, y: event.clientY, active: true };
    servoStatusButton.value?.setPointerCapture(event.pointerId);
  };

  const onServoPointerMove = (event: PointerEvent) => {
    if (!servoPointer || servoPointer.id !== event.pointerId || !servoPointer.active) return;
    const moved = Math.hypot(event.clientX - servoPointer.x, event.clientY - servoPointer.y);
    if (moved < 4) return;
    servoPointer.active = false;
    servoDragOccurred.value = true;
    void getCurrentWindow().startDragging();
  };

  const onServoPointerEnd = (event: PointerEvent) => {
    if (servoPointer?.id === event.pointerId) servoPointer = null;
  };

  onScopeDispose(() => {
    servoPointer = null;
  });
  useEventListener(appIcon, "click", () => (store.state.tab = "home"));
  useEventListener(servoStatusButton, "click", openServoStatus);
  useEventListener(servoStatusButton, "pointerdown", onServoPointerDown);
  useEventListener(servoStatusButton, "pointermove", onServoPointerMove);
  useEventListener(servoStatusButton, "pointerup", onServoPointerEnd);
  useEventListener(servoStatusButton, "pointercancel", onServoPointerEnd);
  useEventListener(settingsButton, "click", (event: MouseEvent) => {
    event.stopPropagation();
    store.state.tab = store.state.tab === "settings" ? "home" : "settings";
  });
  return () => (
    <Titlebar
      min
      max
      close
      v-slots={{
        default: () => (
          <>
            <img
              ref={appIcon}
              class={style.titlebarAppIcon}
              src={appIconUrl}
              alt="microduck-vision"
            />
            {store.state.tab === "settings" ? (
              <div class={style.titlebarMachineTabs} role="tablist" aria-label="设置分类">
                <button
                  ref={(element) => {
                    // oxlint-disable-next-line no-unsafe-type-assertion
                    machineTabButtons.get("simulation")!.value = element as
                      | HTMLButtonElement
                      | undefined;
                  }}
                  class={store.state.machineView === "simulation" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "simulation"}
                >
                  模拟机
                </button>
                <button
                  ref={(element) => {
                    // oxlint-disable-next-line no-unsafe-type-assertion
                    machineTabButtons.get("servo")!.value = element as
                      | HTMLButtonElement
                      | undefined;
                  }}
                  class={store.state.machineView === "servo" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "servo"}
                >
                  舵机
                </button>
                <button
                  ref={(element) => {
                    // oxlint-disable-next-line no-unsafe-type-assertion
                    machineTabButtons.get("physical")!.value = element as
                      | HTMLButtonElement
                      | undefined;
                  }}
                  class={store.state.machineView === "physical" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "physical"}
                >
                  实机
                </button>
              </div>
            ) : (
              <button ref={servoStatusButton} class={style.titlebarServoStatus}>
                <i class={[style.titlebarStatusDot, onlineServos.value ? style.ok : style.off]} />
                舵机 · {onlineServos.value} 在线
                {onlineMachines.value > 1 && ` · ${onlineMachines.value} 台实机`}
              </button>
            )}
          </>
        ),
        actions: () => (
          <button
            ref={settingsButton}
            class={
              store.state.tab === "settings"
                ? [style.titlebarSettingsButton, style.active]
                : style.titlebarSettingsButton
            }
            aria-label="设置"
            title="设置"
          >
            <span class={style.titlebarSettingsIcon} aria-hidden="true">
              {"\uE713"}
            </span>
          </button>
        ),
      }}
    />
  );
});

export const App = defineComponent(() => {
  const store = useDesktopStore();
  const reloadButton = shallowRef<HTMLButtonElement>();
  const appShell = shallowRef<HTMLDivElement>();
  let stopPhysicalMachineEvents: (() => void) | undefined;
  let stopTrainingEvents: (() => void) | undefined;
  let stopTrainingFrameEvents: (() => void) | undefined;
  let stopLoopStatusEvents: (() => void) | undefined;
  let eventListenerDisposed = false;
  useEventListener(window, "error", (event: ErrorEvent) => {
    void commands
      .frontendDiagnostic(
        event.error instanceof Error ? (event.error.stack ?? event.error.message) : event.message,
      )
      .catch(() => undefined);
  });
  useEventListener(window, "unhandledrejection", (event: PromiseRejectionEvent) => {
    void commands
      .frontendDiagnostic(
        event.reason instanceof Error
          ? (event.reason.stack ?? event.reason.message)
          : String(event.reason),
      )
      .catch(() => undefined);
  });

  {
    const hasTauriRuntime = "__TAURI_INTERNALS__" in globalThis || "__TAURI__" in globalThis;
    if (hasTauriRuntime) {
      void listen("physical-machines-changed", () => {
        void Promise.all([refreshPhysicalMachines(), refreshPhysicalPorts()]).catch((error) => {
          if (!eventListenerDisposed) console.error("failed to refresh physical machines", error);
        });
      }).then((unlisten) => {
        if (eventListenerDisposed) unlisten();
        else stopPhysicalMachineEvents = unlisten;
      });
      void listen<TrainingProgress>("training-progress", (event) => {
        if (eventListenerDisposed) return;
        const previous = store.state.training;
        const finished =
          event.payload.phase === "finished" &&
          (previous?.runId !== event.payload.runId || previous.phase !== "finished");
        store.applyTrainingProgress(event.payload);
        if (finished) {
          void commands
            .policyList()
            .then((policies) => {
              if (!eventListenerDisposed) store.state.policies = policies;
            })
            .catch(() => undefined);
        }
      }).then((unlisten) => {
        if (eventListenerDisposed) unlisten();
        else stopTrainingEvents = unlisten;
      });
      void listen<TrainingFrame>("training-frame", (event) => {
        if (!eventListenerDisposed) store.applyTrainingFrame(event.payload);
      }).then((unlisten) => {
        if (eventListenerDisposed) unlisten();
        else stopTrainingFrameEvents = unlisten;
      });
      void listen<LoopStatus>("loop-status", (event) => {
        if (eventListenerDisposed) return;
        if (event.payload.runtimeGeneration !== store.state.loop?.runtimeGeneration) return;
        store.applyLoopStatus(event.payload);
      }).then((unlisten) => {
        if (eventListenerDisposed) unlisten();
        else stopLoopStatusEvents = unlisten;
      });
      void commands
        .loopStatus()
        .then((status) => {
          if (!eventListenerDisposed && status) store.applyLoopStatus(status);
        })
        .catch(() => undefined);
      void commands
        .trainingStatus()
        .then((progress) => {
          if (!eventListenerDisposed && !store.state.training)
            store.applyTrainingProgress(progress);
        })
        .catch(() => undefined);
    }
  }

  onScopeDispose(() => {
    eventListenerDisposed = true;
    stopPhysicalMachineEvents?.();
    stopTrainingEvents?.();
    stopTrainingFrameEvents?.();
    stopLoopStatusEvents?.();
  });
  useEventListener(reloadButton, "click", () => void load());
  useEventListener(appShell, "contextmenu", (event: MouseEvent) => event.preventDefault());

  const page = () => {
    if (store.state.error) {
      return (
        <div class={[style.statusPage, style.statusError]}>
          <h1>读取失败</h1>
          <p>{store.state.error}</p>
          <button ref={reloadButton}>重新加载</button>
        </div>
      );
    }
    if (!store.state.catalog.length && !store.state.error) {
      return <div class={[style.statusPage, style.statusLoading]}>正在读取配置…</div>;
    }
    switch (store.state.tab) {
      case "settings":
        return <SettingsPage />;
      default:
        return null;
    }
  };

  return () => (
    <div ref={appShell} class={style.appShell}>
      <Message />
      <DesktopTitlebar />
      <main class={store.state.tab === "settings" ? style.settingsMain : style.homeMain}>
        <HomePage />
        {page()}
      </main>
    </div>
  );
});
