import { defineComponent, onMounted, ref, watch } from "vue";
import { ScrollArea } from "../../components/scroll-area";
import { useDesktopStore } from "../../store";
import { PhysicalMachinesPage } from "../machines";
import { SimulationPage } from "../simulation";
import { ServoLibraryPage } from "../servo-library";
import { PolicyPage } from "../policy";
import style from "./style.module.scss";

const tabs = [
  { label: "机器", view: "simulation" as const },
  { label: "策略", view: "policy" as const },
];

export const SettingsPage = defineComponent(() => {
  const store = useDesktopStore();
  const collapsed = ref(false);

  onMounted(() => {
    collapsed.value = globalThis.localStorage?.getItem("microduck.settings-collapsed") === "1";
  });
  watch(collapsed, (value) => {
    globalThis.localStorage?.setItem("microduck.settings-collapsed", value ? "1" : "0");
  });

  return () => (
    <div class={[style.settingsPage, collapsed.value && style.collapsed]}>
      <aside class={style.settingsSidebar} aria-label="设置分类">
        <div
          class={style.settingsSidebarTopline}
          role="button"
          tabindex="0"
          aria-label={collapsed.value ? "展开设置导航" : "收起设置导航"}
          title={collapsed.value ? "展开设置导航" : "收起设置导航"}
          onClick={() => (collapsed.value = !collapsed.value)}
          onKeydown={(event: KeyboardEvent) => {
            if (event.key === "Enter" || event.key === " ") {
              event.preventDefault();
              collapsed.value = !collapsed.value;
            }
          }}
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
              type="button"
              class={
                (
                  view === "policy"
                    ? store.state.machineView === "policy"
                    : store.state.machineView !== "policy"
                )
                  ? style.active
                  : undefined
              }
              aria-label={label}
              title={collapsed.value ? label : undefined}
              onClick={() => (store.state.machineView = view)}
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
            {store.state.machineView === "policy" ? (
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
