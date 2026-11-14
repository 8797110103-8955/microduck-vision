import { computed, defineComponent } from "vue";
import type { ServoModelKey } from "../../bridge/types";
import { useDesktopStore } from "../../store";
import style from "./style.module.scss";

const numberText = (value: number | null | undefined) =>
  value == null ? "—" : Number(value.toPrecision(8)).toString();
const valueText = (value: number | string | null | undefined, unit = "") =>
  value == null ? "—" : `${typeof value === "number" ? numberText(value) : value}${unit}`;
const rangeText = (value: [number | null, number | null] | null | undefined, unit = "") =>
  value == null ? "—" : `${valueText(value[0])} – ${valueText(value[1])}${unit}`;

export const ServoLibraryPage = defineComponent(() => {
  const store = useDesktopStore();
  const selected = computed(() =>
    store.state.catalog.find((entry) => entry.key === store.state.selectedServo),
  );
  const hardwareRows = computed(() => {
    const profile = selected.value;
    if (!profile) return [];
    return [
      ["model_number", "型号编号", valueText(profile.modelNumber)],
      ["mass", "舵机重量", valueText(profile.hardwareMassG, " g")],
      ["gear_ratio", "齿轮减速比", valueText(profile.hardwareGearRatio)],
      ["backlash", "齿轮回差", valueText(profile.hardwareBacklashDeg, "°")],
      ["resolution", "编码器分辨率", valueText(profile.resolution, " count/rev")],
      ["voltage_range", "硬件输入电压范围", rangeText(profile.hardwareVoltageRange, " V")],
      ["rated_voltage", "BAM 标称供电电压", valueText(profile.bamVin, " V")],
      [
        "torque_constant_datasheet",
        "规格书转矩电流比",
        valueText(profile.datasheetTorqueConstant, " kg·cm/A"),
      ],
      ["no_load_speed", "4.8V 空载速度", valueText(profile.noLoadSpeedRpm, " RPM")],
      ["peak_stall_torque", "4.8V 堵转扭矩", valueText(profile.peakStallTorqueKgCm, " kg·cm")],
    ];
  });
  const bamRows = computed(() => {
    const profile = selected.value;
    if (!profile) return [];
    return [
      ["actuator", "执行器型号标识", valueText(profile.bamMotor)],
      ["model", "BAM 摩擦模型版本", valueText(profile.bamModel)],
      ["vin", "BAM 标称供电电压", valueText(profile.bamVin, " V")],
      ["kp", "BAM 固件位置增益", valueText(profile.bamKp)],
      ["error_gain", "位置误差到 PWM 换算系数", valueText(profile.bamErrorGain)],
      ["max_pwm", "最大 PWM 占空比", valueText(profile.bamMaxPwm)],
      ["kt", "电机转矩常数", valueText(profile.bamKt, " N·m/A")],
      ["R", "电机绕组电阻", valueText(profile.bamResistance, " Ω")],
      ["armature", "电机等效转动惯量", valueText(profile.bamArmature, " kg·m²")],
      ["q_offset", "位置零点偏移", valueText(profile.bamQOffset, " rad")],
      ["command_delay", "指令延迟", valueText(profile.bamCommandDelay, " s")],
      ["friction_base", "基础静态摩擦", valueText(profile.bamFrictionBase, " N·m")],
      ["friction_stribeck", "Stribeck 静态摩擦项", valueText(profile.bamFrictionStribeck)],
      ["friction_viscous", "黏性摩擦系数", valueText(profile.bamFrictionViscous, " N·m/(rad/s)")],
      ["load_friction_motor", "电机侧负载摩擦系数", valueText(profile.bamLoadFrictionMotor)],
      ["load_friction_external", "输出侧负载摩擦系数", valueText(profile.bamLoadFrictionExternal)],
      [
        "load_friction_motor_stribeck",
        "电机侧 Stribeck 负载项",
        valueText(profile.bamLoadFrictionMotorStribeck),
      ],
      [
        "load_friction_external_stribeck",
        "输出侧 Stribeck 负载项",
        valueText(profile.bamLoadFrictionExternalStribeck),
      ],
      ["load_friction_motor_quad", "电机侧二次摩擦项", valueText(profile.bamLoadFrictionMotorQuad)],
      [
        "load_friction_external_quad",
        "输出侧二次摩擦项",
        valueText(profile.bamLoadFrictionExternalQuad),
      ],
      ["dtheta_stribeck", "Stribeck 速度尺度", valueText(profile.bamDthetaStribeck, " rad/s")],
      ["alpha", "Stribeck 曲线形状参数", valueText(profile.bamAlpha)],
      [
        "XL330_ENCODER_COUNTS_PER_REV",
        "XL330 每圈编码器计数",
        profile.key === "xl330" ? valueText(profile.resolution) : "—",
      ],
      ["XL330_KP_DIVISOR", "XL330 固件增益分频因子", valueText(profile.kpDivisor)],
      ["XL330_PWM_LIMIT", "XL330 默认 PWM 寄存器上限", valueText(profile.pwmLimit)],
    ];
  });
  return () => (
    <div class={[style.page, style.servoLibraryPage]}>
      <div class={[style.modelTabs, style.largeTabs]}>
        {store.state.catalog.map((entry) => (
          <button
            class={store.state.selectedServo === entry.key ? style.active : undefined}
            onClick={() => selectServo(entry.key)}
          >
            {entry.label}
          </button>
        ))}
      </div>
      {selected.value && (
        <div class={style.catalogDetailGrid}>
          <article class={[style.panelCard, style.catalogDetailCard]}>
            <div class={style.cardTopline}>
              <h2>质量与动力学</h2>
              <span class={style.muted}>duck-body</span>
            </div>
            <section class={style.catalogSubsection}>
              <h3>质量与硬件</h3>
              <div class={style.factsList}>
                {hardwareRows.value.map(([key, meaning, value]) => (
                  <div class={style.factRow}>
                    <div class={[style.factKey, style.mono]}>
                      {key}（{meaning}）
                    </div>
                    <div class={style.factValue}>{value}</div>
                  </div>
                ))}
              </div>
            </section>
            <section class={style.catalogSubsection}>
              <h3>BAM M6</h3>
              <div class={style.factsList}>
                {bamRows.value.map(([key, meaning, value]) => (
                  <div class={style.factRow}>
                    <div class={[style.factKey, style.mono]}>
                      {key}（{meaning}）
                    </div>
                    <div class={style.factValue}>{value}</div>
                  </div>
                ))}
              </div>
            </section>
          </article>
          <article class={[style.panelCard, style.catalogDetailCard]}>
            <div class={style.cardTopline}>
              <h2>训练随机化</h2>
            </div>
          </article>
        </div>
      )}
    </div>
  );
});

const selectedServoStorageKey = "microduck.selected-servo";

export function selectServo(model: ServoModelKey) {
  const store = useDesktopStore();
  store.state.selectedServo = model;
  globalThis.localStorage?.setItem(selectedServoStorageKey, model);
}
