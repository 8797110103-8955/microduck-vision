import { defineComponent, onMounted, ref, shallowRef, watch } from "vue";
import { useEventListener } from "@vueuse/core";
import { ScrollArea } from "../../components/scroll-area";
import { useDesktopStore } from "../../store";
import { PhysicalMachinesPage } from "../machines";
import { SimulationPage } from "../simulation";
import { ServoLibraryPage } from "../servo-library";
import { PolicyPage } from "../policy";
import { LogsPage } from "../logs";
import style from "./style.module.scss";

const tabs = [
  { label: "机器", view: "simulation" as const },
  { label: "策略", view: "policy" as const },
  { label: "日志", view: "logs" as const },
];

export const SettingsPage = defineComponent(() => {
  const store = useDesktopStore();
  const collapsed = ref(false);
  const collapseButton = shallowRef<HTMLElement>();
  const tabButtons = new Map<string, ReturnType<typeof shallowRef<HTMLButtonElement>>>();

  for (const { view } of tabs) {
    const button = shallowRef<HTMLButtonElement>();
    tabButtons.set(view, button);
    useEventListener(button, "click", () => (store.state.machineView = view));
  }

  onMounted(() => {
    collapsed.value = globalThis.localStorage?.getItem("microduck.settings-collapsed") === "1";
  });
  watch(collapsed, (value) => {
    globalThis.localStorage?.setItem("microduck.settings-collapsed", value ? "1" : "0");
  });
  useEventListener(collapseButton, "click", () => (collapsed.value = !collapsed.value));
  useEventListener(collapseButton, "keydown", (event: KeyboardEvent) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      collapsed.value = !collapsed.value;
    }
  });
  return () => (
    <div class={[style.settingsPage, collapsed.value && style.collapsed]}>
      <aside class={style.settingsSidebar} aria-label="设置分类">
        <div
          ref={collapseButton}
          class={style.settingsSidebarTopline}
          role="button"
          tabindex="0"
          aria-label={collapsed.value ? "展开设置导航" : "收起设置导航"}
          title={collapsed.value ? "展开设置导航" : "收起设置导航"}
        >
          <span class={style.settingsCollapse} aria-hidden="true">
            <span
              class={
                collapsed.value
                  ? [style.settingsCollapseIcon, style.right]
                  : [style.settingsCollapseIcon, style.left]
              }
            />
          </span>
        </div>
        <nav class={style.settingsNav}>
          {tabs.map(({ label, view }) => (
            <button
              ref={(element) => {
                // oxlint-disable-next-line no-unsafe-type-assertion
                tabButtons.get(view)!.value = element as HTMLButtonElement | undefined;
              }}
              type="button"
              class={
                (
                  view === "simulation"
                    ? !["policy", "logs"].includes(store.state.machineView)
                    : store.state.machineView === view
                )
                  ? style.active
                  : undefined
              }
              aria-label={label}
              title={collapsed.value ? label : undefined}
            >
              {collapsed.value && label.slice(0, 1)}
              <span class={style.settingsNavLabel}>{label}</span>
            </button>
          ))}
        </nav>
      </aside>
      <section class={style.settingsContent}>
        <ScrollArea fill>
          <div class={style.settingsContentInner}>
            {store.state.machineView === "logs" ? (
              <LogsPage />
            ) : store.state.machineView === "policy" ? (
              <PolicyPage />
            ) : store.state.machineView === "simulation" ? (
              <SimulationPage />
            ) : store.state.machineView === "servo" ? (
              <ServoLibraryPage />
            ) : (
              <PhysicalMachinesPage />
            )}
          </div>
        </ScrollArea>
      </section>
    </div>
  );
});
