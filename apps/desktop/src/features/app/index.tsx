import { computed, defineComponent, onUnmounted, ref } from "vue";
import { getCurrentWindow } from "@tauri-apps/api/window";
import appIconUrl from "../../../assets/icons/icon.ico";
import { Titlebar } from "../../components/titlebar";
import { useDesktopStore } from "../../store";
import { load } from "../debug";
import { HomePage } from "../home";
import { SettingsPage } from "../settings";
import style from "./style.module.scss";

const DesktopTitlebar = defineComponent(() => {
  const store = useDesktopStore();
  const servoDragOccurred = ref(false);
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
    if (event.currentTarget instanceof HTMLElement) {
      event.currentTarget.setPointerCapture(event.pointerId);
    }
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

  onUnmounted(() => {
    servoPointer = null;
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
              class={style.titlebarAppIcon}
              onClick={() => (store.state.tab = "home")}
              src={appIconUrl}
              alt="microduck-vision"
            />
            {store.state.tab === "settings" ? (
              <div class={style.titlebarMachineTabs} role="tablist" aria-label="设置分类">
                <button
                  class={store.state.machineView === "simulation" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "simulation"}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    store.state.machineView = "simulation";
                  }}
                >
                  模拟机
                </button>
                <button
                  class={store.state.machineView === "servo" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "servo"}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    store.state.machineView = "servo";
                  }}
                >
                  舵机
                </button>
                <button
                  class={store.state.machineView === "physical" ? style.active : undefined}
                  role="tab"
                  aria-selected={store.state.machineView === "physical"}
                  onClick={(event: MouseEvent) => {
                    event.stopPropagation();
                    store.state.machineView = "physical";
                  }}
                >
                  实机
                </button>
              </div>
            ) : (
              <button
                class={style.titlebarServoStatus}
                onClick={openServoStatus}
                onPointerdown={onServoPointerDown}
                onPointermove={onServoPointerMove}
                onPointerup={onServoPointerEnd}
                onPointercancel={onServoPointerEnd}
              >
                <i class={[style.titlebarStatusDot, onlineServos.value ? style.ok : style.off]} />
                舵机 · {onlineServos.value} 在线
                {onlineMachines.value > 1 && ` · ${onlineMachines.value} 台实机`}
              </button>
            )}
          </>
        ),
        actions: () => (
          <button
            class={
              store.state.tab === "settings"
                ? [style.titlebarSettingsButton, style.active]
                : style.titlebarSettingsButton
            }
            aria-label="设置"
            title="设置"
            onClick={(event: MouseEvent) => {
              event.stopPropagation();
              store.state.tab = store.state.tab === "settings" ? "home" : "settings";
            }}
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

  const page = () => {
    if (store.state.error) {
      return (
        <div class={[style.statusPage, style.statusError]}>
          <h1>读取失败</h1>
          <p>{store.state.error}</p>
          <button onClick={() => void load()}>重新加载</button>
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
    <div class={style.appShell} onContextmenu={(event: MouseEvent) => event.preventDefault()}>
      <DesktopTitlebar />
      <main class={store.state.tab === "settings" ? style.settingsMain : style.homeMain}>
        <HomePage />
        {page()}
      </main>
    </div>
  );
});
