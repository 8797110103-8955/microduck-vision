// UI test double only. These synthesized states are not a physics or policy reference.
import type {
  ActionRecord,
  LoopAction,
  LoopStatus,
  RuntimeCommand,
  RuntimeOpenInput,
  PhysicalMachine,
  PolicyEntry,
  PreviewInput,
  PhysicalMachineDetectInput,
  PhysicalMachineProbeInput,
  PhysicalMachineReconnectInput,
  PhysicalMachineSaveInput,
  PhysicalMachineInput,
  PhysicalServoRegister,
  SceneInfo,
  SceneFrame,
  ServoCatalogEntry,
  ServoModelKey,
  TrainingProgress,
  TrainingRunMetadata,
  TrainingStartInput,
  VirtualMachine,
} from "../../generated/types";

function simulationFor(catalog: ServoCatalogEntry[], model: ServoModelKey) {
  const entry = catalog.find((item) => item.key === model) ?? catalog[0];
  if (!entry) throw new Error(`Missing servo profile for ${model}`);
  return {
    damping: entry.simDamping,
    frictionLoss: entry.simFrictionLoss,
    armature: entry.simArmature,
    kp: entry.simKp,
    forceRange: entry.simForceRange,
    worldPosition: [0, 0, 0] as [number, number, number],
  };
}

let memoryTrainingRunSequence = 0;

function sameCensus(a: PhysicalMachine["census"], b: PhysicalMachine["census"]) {
  const leftCensus = a ?? [];
  const rightCensus = b ?? [];
  if (!leftCensus.length || leftCensus.length !== rightCensus.length) return false;
  const left = leftCensus
    .map((servo) => `${servo.servoId}:${servo.model.trim().toLowerCase()}`)
    .toSorted();
  const right = rightCensus
    .map((servo) => `${servo.servoId}:${servo.model.trim().toLowerCase()}`)
    .toSorted();
  return left.every((value, index) => value === right[index]);
}

function censusFingerprint(census: NonNullable<PhysicalMachine["census"]>) {
  return md5Hex(
    census
      .toSorted((a, b) => a.servoId - b.servoId)
      .map((servo) => `${servo.servoId}:${servo.model.trim().toLowerCase()}`)
      .join("\0"),
  );
}

const md5Rotate = (value: number, count: number) => (value << count) | (value >>> (32 - count));
const md5Little = (value: number) =>
  Array.from({ length: 4 }, (_, index) => (value >>> (8 * index)) & 0xff);
const md5Add = (...values: number[]) => values.reduce((sum, value) => (sum + value) >>> 0, 0);

function md5Hex(text: string): string {
  const source = Array.from(new TextEncoder().encode(text));
  const bitLength = source.length * 8;
  source.push(0x80);
  while (source.length % 64 !== 56) source.push(0);
  for (let index = 0; index < 8; index += 1) source.push((bitLength / 2 ** (8 * index)) & 0xff);
  let a0 = 0x67452301;
  let b0 = 0xefcdab89;
  let c0 = 0x98badcfe;
  let d0 = 0x10325476;
  const shifts = [7, 12, 17, 22, 5, 9, 14, 20, 4, 11, 16, 23, 6, 10, 15, 21];
  const constants = Array.from(
    { length: 64 },
    (_, index) => Math.floor(Math.abs(Math.sin(index + 1)) * 2 ** 32) >>> 0,
  );
  for (let offset = 0; offset < source.length; offset += 64) {
    const words = Array.from({ length: 16 }, (_, index) => {
      const base = offset + index * 4;
      return (
        (source[base] |
          (source[base + 1] << 8) |
          (source[base + 2] << 16) |
          (source[base + 3] << 24)) >>>
        0
      );
    });
    let a = a0;
    let b = b0;
    let c = c0;
    let d = d0;
    for (let index = 0; index < 64; index += 1) {
      let f: number;
      let g: number;
      if (index < 16) {
        f = (b & c) | (~b & d);
        g = index;
      } else if (index < 32) {
        f = (d & b) | (~d & c);
        g = (5 * index + 1) % 16;
      } else if (index < 48) {
        f = b ^ c ^ d;
        g = (3 * index + 5) % 16;
      } else {
        f = c ^ (b | ~d);
        g = (7 * index) % 16;
      }
      const next = d;
      d = c;
      c = b;
      b = md5Add(
        b,
        md5Rotate(
          md5Add(a, f, constants[index], words[g]),
          shifts[Math.floor(index / 16) * 4 + (index % 4)],
        ),
      );
      a = next;
    }
    a0 = (a0 + a) >>> 0;
    b0 = (b0 + b) >>> 0;
    c0 = (c0 + c) >>> 0;
    d0 = (d0 + d) >>> 0;
  }
  return [...md5Little(a0), ...md5Little(b0), ...md5Little(c0), ...md5Little(d0)]
    .map((byte) => byte.toString(16).padStart(2, "0"))
    .join("");
}

const browserCatalog: ServoCatalogEntry[] = (["hd1910", "xl330"] as ServoModelKey[]).map((key) => ({
  key,
  label: key === "hd1910" ? "飞特 HD1910" : "XL330",
  protocol: "browser",
  defaultBinding: key === "hd1910",
  capabilities: [],
  modelNumber: 0,
  resolution: 0,
  kpDivisor: null,
  pwmLimit: null,
  defaultBaud: 0,
  baudRates: [],
  registerCount: 0,
  supportsFastSync: false,
  supportsCenterCalibration: false,
  resetNeedsUnlock: false,
  supportsReboot: false,
  protocolDefinition: "browser",
  velocityRadPerCount: null,
  voltsPerCount: null,
  milliampsPerCount: null,
  moveBlock: [],
  massG: null,
  baselineMassG: null,
  hardwareMassG: null,
  hardwareGearRatio: null,
  hardwareBacklashDeg: null,
  hardwareVoltageRange: [null, null],
  datasheetTorqueConstant: null,
  noLoadSpeedRpm: null,
  peakStallTorqueKgCm: null,
  bamMotor: "browser",
  bamModel: "browser",
  bamVin: null,
  bamKp: null,
  bamErrorGain: null,
  bamMaxPwm: null,
  bamKt: null,
  bamResistance: null,
  bamArmature: 0.0018,
  bamQOffset: null,
  bamCommandDelay: null,
  bamFrictionBase: null,
  bamFrictionStribeck: null,
  bamFrictionViscous: null,
  bamLoadFrictionMotor: null,
  bamLoadFrictionExternal: null,
  bamLoadFrictionMotorStribeck: null,
  bamLoadFrictionExternalStribeck: null,
  bamLoadFrictionMotorQuad: null,
  bamLoadFrictionExternalQuad: null,
  bamDthetaStribeck: null,
  bamAlpha: null,
  kpFw: null,
  vinRange: [null, null],
  vinDropGainRange: [null, null],
  vinMin: null,
  delaySteps: [0, 0],
  backlashDeg: null,
  pAtKpFw: null,
  simDamping: 0.053,
  simFrictionLoss: 0.0048,
  simArmature: 0.0018,
  simKp: 0.55,
  simForceRange: 0.96,
}));

export class MemoryBackend {
  readonly calls: Array<{ name: string; value?: unknown }> = [];
  private machines: VirtualMachine[];
  private physical: PhysicalMachine[] = [];
  private autoReconnect = true;
  private actionList: ActionRecord[] = [
    { id: "stand", name: "站立", kind: "pose", updatedAt: "内置", source: "builtin" },
    { id: "sit", name: "坐下", kind: "pose", updatedAt: "内置", source: "builtin" },
  ];
  private status: LoopStatus | null = null;
  private previewStatus: LoopStatus | null = null;
  private previewInput: PreviewInput = {
    twist: [0, 0, 0],
    head: [0, 0, 0, 0],
    body: [0, 0, 0],
    targetDistanceM: null,
  };
  private previewCommandEncoding: string | null = null;
  private trainingProgress: TrainingProgress | null = null;
  private readonly policyFiles = [
    "alpha_walking.onnx",
    "velstand.onnx",
    "alpha_stand.onnx",
    "roller.onnx",
    "alpha_sitstand.onnx",
    "alpha_ground_pick.onnx",
    "roller_crouch.onnx",
    "roulade.onnx",
    "ball_kick_left.onnx",
    "ball_kick_right.onnx",
  ];
  private readonly policyState = new Map<
    string,
    { cached: boolean; selected: boolean; size: number }
  >();

  private readonly catalog: ServoCatalogEntry[];

  constructor(catalog: ServoCatalogEntry[]) {
    this.catalog = catalog.length ? catalog : browserCatalog;
    this.machines = [
      {
        id: "official-hd1910",
        name: "机器 0",
        bodyId: "official",
        servoModel: "hd1910",
        simulation: simulationFor(this.catalog, "hd1910"),
        startupSelected: true,
        referencedBy: [],
      },
    ];
  }

  async officialSceneModel(): Promise<SceneInfo> {
    this.calls.push({ name: "officialSceneModel" });
    return {
      meshDir: "",
      defaultFrame: { bodies: [[0, 0, 0, 1, 0, 0, 0]], markers: [] },
      model: {
        bodies: [{ name: "official-duck", geoms: [] }],
        joints: [],
        trunkHeight: 0.2,
        headCamera: null,
      },
    };
  }

  async officialSceneFrame(positions: number[]): Promise<SceneFrame> {
    this.calls.push({ name: "officialSceneFrame", value: positions });
    return { bodies: [[0, 0, 0, 1, 0, 0, 0]], markers: [] };
  }

  async servoCatalog() {
    this.calls.push({ name: "servoCatalog" });
    return this.catalog.map((entry) => ({ ...entry, capabilities: [...entry.capabilities] }));
  }

  async officialJointSlots() {
    this.calls.push({ name: "officialJointSlots" });
    return [
      "left_hip_yaw",
      "left_hip_roll",
      "left_hip_pitch",
      "left_knee",
      "left_ankle",
      "neck_pitch",
      "head_pitch",
      "head_yaw",
      "head_roll",
      "mouth",
      "right_hip_yaw",
      "right_hip_roll",
      "right_hip_pitch",
      "right_knee",
      "right_ankle",
    ];
  }

  async officialJointRanges() {
    this.calls.push({ name: "officialJointRanges" });
    return (await this.officialJointSlots()).map((slotId) => ({
      slotId,
      minRad: -Math.PI,
      maxRad: Math.PI,
    }));
  }

  async openLogs() {
    this.calls.push({ name: "openLogs" });
  }

  async virtualMachines() {
    this.calls.push({ name: "virtualMachines" });
    return this.machines.map((machine) => ({
      ...machine,
      jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
      referencedBy: [...machine.referencedBy],
    }));
  }

  async physicalMachines() {
    this.calls.push({ name: "physicalMachines" });
    return this.physical.map((machine) => ({
      ...machine,
      jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
      onlineServoIds: [...machine.onlineServoIds],
    }));
  }

  async physicalPorts() {
    this.calls.push({ name: "physicalPorts" });
    return [
      { portName: "COM1", vid: null, pid: null, adapterSerial: null, product: "Memory port" },
    ];
  }

  async autoReconnectEnabled() {
    this.calls.push({ name: "autoReconnectEnabled" });
    return this.autoReconnect;
  }

  async setAutoReconnectEnabled(enabled: boolean) {
    this.calls.push({ name: "setAutoReconnectEnabled", value: enabled });
    this.autoReconnect = enabled;
    return enabled;
  }

  async physicalBaudChoices() {
    this.calls.push({ name: "physicalBaudChoices" });
    return [1_000_000, 576_000, 115_200, 57_600];
  }

  async probePhysicalMachine(input: PhysicalMachineProbeInput) {
    this.calls.push({ name: "probePhysicalMachine", value: input });
    return this.detectPhysicalMachine({
      census: [
        { servoId: 1, model: "hd1910" },
        { servoId: 2, model: "hd1910" },
      ],
      reconnectHint: input.reconnectHint,
    });
  }

  async detectPhysicalMachine(input: PhysicalMachineDetectInput) {
    this.calls.push({ name: "detectPhysicalMachine", value: input });
    const census = input.census
      .map((servo) => ({ servoId: servo.servoId, model: servo.model.trim().toLowerCase() }))
      .toSorted((a, b) => a.servoId - b.servoId);
    const valid =
      census.length > 0 &&
      census.every(
        (servo, index) =>
          (servo.model === "hd1910" || servo.model === "xl330") &&
          (index === 0 || census[index - 1].servoId !== servo.servoId),
      );
    const md5 = valid ? censusFingerprint(census) : null;
    const matches = valid
      ? this.physical.filter((machine) => sameCensus(machine.census, census))
      : [];
    return {
      census,
      md5,
      reconnectHint: input.reconnectHint,
      matchResult: !valid
        ? ("incompleteCensus" as const)
        : matches.length > 1
          ? ("identityConflict" as const)
          : matches.length === 1
            ? { unique: { physical_machine_id: matches[0].id } }
            : ("new" as const),
      diagnostics: !valid
        ? ["IncompleteCensus" as const]
        : matches.length > 1
          ? ["IdentityConflict" as const]
          : [],
    };
  }

  async savePhysicalMachine(input: PhysicalMachineSaveInput) {
    this.calls.push({ name: "savePhysicalMachine", value: input });
    const detected = await this.detectPhysicalMachine({
      census: input.census,
      reconnectHint: input.reconnectHint,
    });
    if (!detected.md5 || detected.matchResult !== "new")
      throw new Error("physical machine identity is not a unique new census");
    const machine: PhysicalMachine = {
      id: `physical-${this.physical.length + 1}`,
      name: input.name?.trim() || detected.md5,
      usbPort: input.reconnectHint?.portName ?? "",
      machineCode: detected.md5,
      census: detected.census,
      reconnectHint: input.reconnectHint,
      virtualMachineId: null,
      onlineServoIds: detected.census.map((servo) => servo.servoId),
      connected: true,
      lastSeen: "刚刚",
      jointBindings: [],
    };
    this.physical.push(machine);
    return {
      ...machine,
      census: [...(machine.census ?? [])],
      jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
      onlineServoIds: [...machine.onlineServoIds],
    };
  }

  async disconnectPhysicalMachine(id: string) {
    this.calls.push({ name: "disconnectPhysicalMachine", value: id });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    machine.connected = false;
    return {
      ...machine,
      census: [...(machine.census ?? [])],
      jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
      onlineServoIds: [...machine.onlineServoIds],
    };
  }

  async reconnectPhysicalMachine(input: PhysicalMachineReconnectInput) {
    this.calls.push({ name: "reconnectPhysicalMachine", value: input });
    const machine = this.physical.find((entry) => entry.id === input.id);
    if (!machine) throw new Error(`unknown physical machine: ${input.id}`);
    if (!sameCensus(machine.census, input.census)) {
      machine.connected = false;
      throw new Error("reconnect census does not match saved physical machine");
    }
    machine.connected = true;
    machine.lastSeen = "刚刚";
    machine.reconnectHint = input.reconnectHint;
    machine.usbPort = input.reconnectHint?.portName ?? machine.usbPort;
    machine.machineCode = censusFingerprint(machine.census ?? []);
    return {
      ...machine,
      census: [...(machine.census ?? [])],
      onlineServoIds: [...machine.onlineServoIds],
    };
  }

  async autoReconnectPhysicalMachine(id: string, baud: number) {
    this.calls.push({ name: "autoReconnectPhysicalMachine", value: { id, baud } });
    return this.reconnectPhysicalMachine({
      id,
      census: this.physical.find((machine) => machine.id === id)?.census ?? [],
      reconnectHint: this.physical.find((machine) => machine.id === id)?.reconnectHint ?? null,
    });
  }

  async createPhysicalMachine(input: PhysicalMachineInput) {
    const machine: PhysicalMachine = {
      ...input,
      census: [],
      reconnectHint: input.usbPort
        ? { portName: input.usbPort, vid: null, pid: null, adapterSerial: null, product: null }
        : null,
      id: `physical-${this.physical.length + 1}`,
      virtualMachineId: null,
      onlineServoIds: [],
      connected: false,
      lastSeen: null,
      jointBindings: [],
    };
    this.calls.push({ name: "createPhysicalMachine", value: machine });
    this.physical.push(machine);
    return { ...machine };
  }

  async connectPhysicalMachine(id: string) {
    this.calls.push({ name: "connectPhysicalMachine", value: id });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    machine.connected = true;
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async mountPhysicalMachine(id: string, virtualMachineId: string) {
    this.calls.push({ name: "mountPhysicalMachine", value: { id, virtualMachineId } });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    if (!this.machines.some((entry) => entry.id === virtualMachineId))
      throw new Error("unknown virtual machine");
    machine.virtualMachineId = virtualMachineId;
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async deletePhysicalMachine(id: string) {
    this.calls.push({ name: "deletePhysicalMachine", value: id });
    this.physical = this.physical.filter((machine) => machine.id !== id);
  }

  async renamePhysicalMachine(id: string, name: string) {
    this.calls.push({ name: "renamePhysicalMachine", value: { id, name } });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    machine.name = name;
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async bindPhysicalMachine(input: {
    physicalMachineId: string;
    slotId: string;
    direction: number;
  }) {
    this.calls.push({ name: "bindPhysicalMachine", value: input });
    const machine = this.physical.find((entry) => entry.id === input.physicalMachineId);
    if (!machine) throw new Error(`unknown physical machine: ${input.physicalMachineId}`);
    if (input.direction != null && input.direction !== 1 && input.direction !== -1)
      throw new Error(`invalid joint direction: ${input.direction}`);
    if (input.direction != null) {
      machine.jointDirections = [
        ...(machine.jointDirections ?? []).filter((binding) => binding.slotId !== input.slotId),
        { slotId: input.slotId, direction: input.direction },
      ];
    }
    return {
      ...machine,
      jointBindings: machine.jointBindings?.map((binding) => ({ ...binding })),
      onlineServoIds: [...machine.onlineServoIds],
    };
  }

  async bindVirtualMachine(input: {
    virtualMachineId: string;
    slotId: string;
    physicalId: number | null;
  }) {
    this.calls.push({ name: "bindVirtualMachine", value: input });
    const machine = this.machines.find((entry) => entry.id === input.virtualMachineId);
    if (!machine) throw new Error(`unknown virtual machine: ${input.virtualMachineId}`);
    const bindings = (machine.jointBindings ?? []).filter(
      (binding) => binding.slotId !== input.slotId,
    );
    if (input.physicalId !== null) {
      bindings.push({
        slotId: input.slotId,
        physicalId: input.physicalId,
      });
    }
    if (new Set(bindings.map((binding) => binding.physicalId)).size !== bindings.length)
      throw new Error("duplicate servo ID");
    machine.jointBindings = bindings;
    return { ...machine, jointBindings: bindings.map((binding) => ({ ...binding })) };
  }

  async validateVirtualMachineBindings(id: string) {
    this.calls.push({ name: "validateVirtualMachineBindings", value: id });
    const machine = this.machines.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown virtual machine: ${id}`);
    const bindings = machine.jointBindings ?? [];
    const conflictingIds = bindings
      .filter((binding, index) =>
        bindings.slice(index + 1).some((other) => other.physicalId === binding.physicalId),
      )
      .map((binding) => binding.physicalId);
    machine.jointBindings = bindings.filter(
      (binding) => !conflictingIds.includes(binding.physicalId),
    );
    return {
      ...machine,
      jointBindings: machine.jointBindings.map((binding) => ({ ...binding })),
    };
  }

  async validatePhysicalMachineBindings(id: string) {
    this.calls.push({ name: "validatePhysicalMachineBindings", value: id });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async setPhysicalMachinePoseZero(id: string, _virtualMachineId?: string | null) {
    this.calls.push({ name: "setPhysicalMachinePoseZero", value: id });
    const machine = this.physical.find((item) => item.id === id);
    if (!machine) throw new Error("unknown physical machine");
    machine.jointBindings = (machine.jointBindings ?? []).map((binding) => ({
      ...binding,
      zeroOffsetRad: 0,
    }));
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async physicalMachinePose(id: string, _virtualMachineId?: string | null) {
    this.calls.push({ name: "physicalMachinePose", value: id });
    if (!this.physical.some((machine) => machine.id === id))
      throw new Error("unknown physical machine");
    return Array.from({ length: 15 }, () => 0);
  }

  async setPhysicalMachineTorque(id: string, enabled: boolean, _virtualMachineId?: string | null) {
    this.calls.push({ name: "setPhysicalMachineTorque", value: { id, enabled } });
    const machine = this.physical.find((item) => item.id === id);
    if (!machine) throw new Error("unknown physical machine");
    machine.torqueEnabled = enabled;
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async writePhysicalMachinePosition(input: {
    physicalMachineId: string;
    servoId: number;
    positionRad: number;
  }) {
    this.calls.push({ name: "writePhysicalMachinePosition", value: input });
    const machine = this.physical.find((item) => item.id === input.physicalMachineId);
    if (!machine) throw new Error("unknown physical machine");
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async scanPhysicalMachine(id: string) {
    this.calls.push({ name: "scanPhysicalMachine", value: id });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    machine.onlineServoIds = machine.census?.length
      ? machine.census.map((servo) => servo.servoId)
      : [1, 2];
    return { ...machine, onlineServoIds: [...machine.onlineServoIds] };
  }

  async physicalMachineRegisters(id: string): Promise<PhysicalServoRegister[]> {
    this.calls.push({ name: "physicalMachineRegisters", value: id });
    const machine = this.physical.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown physical machine: ${id}`);
    return (machine.census ?? []).flatMap((servo) => [
      {
        servoId: servo.servoId,
        address: 0,
        name: "model_number",
        size: 2,
        default: false,
        writable: false,
        value: 0,
      },
      {
        servoId: servo.servoId,
        address: 40,
        name: "torque_enable",
        size: 1,
        default: false,
        writable: true,
        value: machine.torqueEnabled ? 1 : 0,
      },
      {
        servoId: servo.servoId,
        address: 56,
        name: "present_position",
        size: 2,
        default: false,
        writable: false,
        value: 0,
      },
    ]);
  }

  async readPhysicalMachineRegister(id: string, servoId: number, name: string) {
    this.calls.push({ name: "readPhysicalMachineRegister", value: { id, servoId, name } });
    const machine = this.physical.find((item) => item.id === id);
    if (!machine) throw new Error("unknown physical machine");
    const row = (await this.physicalMachineRegisters(id)).find(
      (entry) => entry.servoId === servoId && entry.name === name,
    );
    if (!row) throw new Error(`unknown register: ${name}`);
    return row;
  }

  async writePhysicalMachineRegister(input: {
    physicalMachineId: string;
    servoId: number;
    name: string;
    value: number;
  }): Promise<PhysicalServoRegister> {
    this.calls.push({ name: "writePhysicalMachineRegister", value: input });
    const machine = this.physical.find((entry) => entry.id === input.physicalMachineId);
    if (!machine) throw new Error(`unknown physical machine: ${input.physicalMachineId}`);
    if (input.name !== "torque_enable") throw new Error("register is read-only");
    machine.torqueEnabled = input.value !== 0;
    return {
      servoId: input.servoId,
      address: 40,
      name: input.name,
      size: 1,
      default: false,
      writable: true,
      value: input.value,
    };
  }

  async policyList() {
    this.calls.push({ name: "policyList" });
    return this.policyEntries();
  }

  async policyDownload(id: string) {
    this.calls.push({ name: "policyDownload", value: id });
    const policy = this.policyEntries().find((entry) => entry.id === id);
    if (!policy) throw new Error(`unknown policy: ${id}`);
    this.policyState.set(policy.file, { cached: true, selected: true, size: 42 });
    this.policyFiles.forEach((file) => {
      if (file !== policy.file) {
        const state = this.policyState.get(file);
        if (state) this.policyState.set(file, { ...state, selected: false });
      }
    });
    return this.policyEntries().find((entry) => entry.id === id)!;
  }

  async policySelect(id: string) {
    this.calls.push({ name: "policySelect", value: id });
    const policy = this.policyEntries().find((entry) => entry.id === id);
    if (!policy?.cached) throw new Error("请先下载这个官方策略");
    this.policyFiles.forEach((file) => {
      const state = this.policyState.get(file);
      if (state) this.policyState.set(file, { ...state, selected: file === policy.file });
    });
    return this.policyEntries().find((entry) => entry.id === id)!;
  }

  private policyEntries(): PolicyEntry[] {
    return this.policyFiles.map((file) => {
      const state = this.policyState.get(file) ?? { cached: false, selected: false, size: 0 };
      const id = file.slice(0, -".onnx".length).replaceAll("_", "-");
      return {
        id,
        name: file,
        file,
        sourceUrl: `https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/${file}`,
        ...state,
      };
    });
  }

  async policyOpenFolder() {
    this.calls.push({ name: "policyOpenFolder" });
  }

  async trainingStart(input: TrainingStartInput) {
    this.calls.push({ name: "trainingStart", value: input });
    if (!input.virtualMachineId) throw new Error("训练必须选择模拟机");
    if (!input.recipe.trim()) throw new Error("训练 recipe 不能为空");
    if (!input.teacherMotionProvenance.trim())
      throw new Error("训练 teacher/motion provenance 不能为空");
    const runId = `memory-training-${++memoryTrainingRunSequence}`;
    const policy = input.policyId
      ? this.policyEntries().find((entry) => entry.id === input.policyId)
      : undefined;
    const metadata: TrainingRunMetadata = {
      experimentId: input.experimentId?.trim() || runId,
      bodyTemplateId: "official",
      bodyVersion: 1,
      bodySourceHash: "memory",
      policyId: input.policyId ?? null,
      policyLayout: {
        obsLen: 61,
        actionLen: 14,
        controlHz: 50,
        commandEncoding: policy?.command?.encoding ?? null,
      },
      servoProfile: "hd1910",
      recipe: input.recipe.trim(),
      teacherMotionProvenance: input.teacherMotionProvenance.trim(),
    };
    this.trainingProgress = {
      runId,
      resultDir: `memory://training/${runId}`,
      metadata,
      phase: "starting",
      step: null,
      totalSteps: null,
      percent: null,
      reward: null,
      loss: null,
      message: "模拟训练已启动",
      line: "",
      exitCode: null,
    };
    return { ...this.trainingProgress };
  }

  async trainingStop() {
    this.calls.push({ name: "trainingStop" });
    if (this.trainingProgress) {
      this.trainingProgress = { ...this.trainingProgress, phase: "stopped", message: "训练已停止" };
    }
  }

  async trainingStatus() {
    this.calls.push({ name: "trainingStatus" });
    return this.trainingProgress ? { ...this.trainingProgress } : null;
  }

  async createVirtualMachine(
    input: Pick<VirtualMachine, "name" | "bodyId" | "servoModel" | "simulation">,
  ) {
    const machine: VirtualMachine = {
      ...input,
      name: input.name.trim() || `机器 ${this.machines.length}`,
      id: `virtual-${this.machines.length + 1}`,
      startupSelected: false,
      referencedBy: [],
    };
    this.calls.push({ name: "createVirtualMachine", value: machine });
    this.machines.push(machine);
    return { ...machine, referencedBy: [] };
  }

  async setStartupVirtualMachine(id: string) {
    this.calls.push({ name: "setStartupVirtualMachine", value: id });
    this.machines = this.machines.map((machine) => ({
      ...machine,
      startupSelected: machine.id === id,
    }));
  }

  async setVirtualMachineWorldPosition(id: string, position: [number, number, number]) {
    this.calls.push({ name: "setVirtualMachineWorldPosition", value: { id, position } });
    const machine = this.machines.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown virtual machine: ${id}`);
    machine.simulation.worldPosition = [...position] as [number, number, number];
    return { ...machine, referencedBy: [...machine.referencedBy] };
  }

  async deleteVirtualMachine(id: string) {
    this.calls.push({ name: "deleteVirtualMachine", value: id });
    this.machines = this.machines.filter((machine) => machine.id !== id);
  }

  async renameVirtualMachine(id: string, name: string) {
    this.calls.push({ name: "renameVirtualMachine", value: { id, name } });
    const machine = this.machines.find((entry) => entry.id === id);
    if (!machine) throw new Error(`unknown virtual machine: ${id}`);
    machine.name = name;
    return { ...machine, referencedBy: [...machine.referencedBy] };
  }

  async actions() {
    this.calls.push({ name: "actions" });
    return this.actionList.map((action) => ({ ...action }));
  }

  async createAction(input: Pick<ActionRecord, "name" | "kind">) {
    const action: ActionRecord = {
      ...input,
      id: `action-${this.actionList.length + 1}`,
      updatedAt: "刚刚",
      source: "user",
    };
    this.calls.push({ name: "createAction", value: action });
    this.actionList.push(action);
    return { ...action };
  }

  async deleteAction(id: string) {
    this.calls.push({ name: "deleteAction", value: id });
    const action = this.actionList.find((entry) => entry.id === id);
    if (action?.source === "builtin") throw new Error("内置动作不能删除");
    this.actionList = this.actionList.filter((entry) => entry.id !== id);
  }

  async loopOpen(input: RuntimeOpenInput & { physicalMachineId: string }) {
    this.calls.push({ name: "loopOpen", value: input });
    const seen = new Set<number>();
    const duplicate = this.machines
      .find((entry) => entry.id === input.virtualMachineId)
      ?.jointBindings?.find((binding) => {
        if (seen.has(binding.physicalId)) return true;
        seen.add(binding.physicalId);
        return false;
      });
    if (duplicate) throw new Error(`绑定冲突：舵机 ${duplicate.physicalId} 被重复绑定`);
    const count = 15;
    this.status = {
      runtimeState: "running",
      hasExecution: true,
      activeBody: { virtualMachineId: null, physicalMachineId: input.physicalMachineId },
      simulation: null,
      readSources: [],
      runtimeGeneration: 1,
      executionGeneration: 1,
      configRevision: 1,
      phase: "limp",
      session: 0,
      tick: 0,
      torque: false,
      positions: Array(count).fill(0),
      targets: Array(count).fill(0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
      jointRangeStates: [],
    };
    return this.status;
  }

  async runtimeOpen(input: RuntimeOpenInput) {
    this.calls.push({ name: "runtimeOpen", value: input });
    if (input.physicalMachineId) {
      return this.loopOpen({
        virtualMachineId: input.virtualMachineId,
        physicalMachineId: input.physicalMachineId,
        access: input.access,
        policyId: input.policyId,
      });
    }
    return this.previewLoopOpen(input.virtualMachineId);
  }

  async runtimeReconfigure(generation: number, configRevision: number, input: RuntimeOpenInput) {
    this.calls.push({ name: "runtimeReconfigure", value: { generation, configRevision, input } });
    this.status = null;
    this.previewStatus = null;
    return this.runtimeOpen(input);
  }

  async runtimeCommand(_generation: number, _executionGeneration: number, request: RuntimeCommand) {
    this.calls.push({ name: "runtimeCommand", value: request });
    if (request.kind === "input") {
      if (this.previewStatus) return this.previewLoopCommand(request.input);
      await this.loopPolicyCommand(request.input);
      return (
        this.status ??
        (() => {
          throw new Error("loop is closed");
        })()
      );
    }
    if (request.kind === "policy") {
      await this.previewLoopPolicy(request.policy_id);
      return this.previewLoopCommand(request.input);
    }
    if (request.kind === "resetWorld") return this.previewLoopResetWorld();
    if (request.kind === "action") return this.loopAction(request.action);
    this.calls.push({ name: "loopCommand", value: request.targets });
    await this.loopCommand(request.targets);
    return (
      this.status ??
      (() => {
        throw new Error("loop is closed");
      })()
    );
  }

  async loopCommand(targets: number[]) {
    this.calls.push({ name: "loopCommand", value: targets });
    if (!this.status) throw new Error("loop is closed");
    this.status = { ...this.status, targets: [...targets] };
  }

  async loopCommandAt(generation: number, targets: number[]) {
    this.calls.push({ name: "loopCommandAt", value: { generation, targets } });
    return this.loopCommand(targets);
  }

  async loopPolicyCommand(input: PreviewInput) {
    this.calls.push({ name: "loopPolicyCommand", value: input });
  }

  async loopPolicyCommandAt(generation: number, input: PreviewInput) {
    this.calls.push({ name: "loopPolicyCommandAt", value: { generation, input } });
    return this.loopPolicyCommand(input);
  }

  async loopAction(action: LoopAction) {
    this.calls.push({ name: "loopAction", value: action });
    if (!this.status) throw new Error("loop is closed");
    if (action.kind === "power") {
      this.status = { ...this.status, phase: action.on ? "homing" : "limp", torque: action.on };
    } else if (
      action.kind === "drive" &&
      action.on &&
      (this.status.phase === "ready" || this.status.phase === "holding")
    ) {
      this.status = { ...this.status, phase: "driving" };
    } else if (action.kind === "drive" && !action.on && this.status.phase === "driving") {
      this.status = { ...this.status, phase: "holding" };
    } else if (action.kind === "stop") {
      this.status = { ...this.status, phase: "stopped", torque: false };
    } else if (action.kind === "estop") {
      this.status = { ...this.status, phase: "estop", torque: false };
    } else if (action.kind === "reconnect") {
      this.status = {
        ...this.status,
        phase: "reconnecting",
        session: action.session,
        torque: false,
      };
    }
    return this.status;
  }

  async loopActionAt(generation: number, action: LoopAction) {
    this.calls.push({ name: "loopActionAt", value: { generation, action } });
    return this.loopAction(action);
  }

  async loopTick() {
    this.calls.push({ name: "loopTick" });
    if (!this.status) throw new Error("loop is closed");
    const phase =
      this.status.phase === "homing" && this.status.tick >= 4 ? "ready" : this.status.phase;
    this.status = {
      ...this.status,
      phase,
      tick: this.status.tick + 1,
      positions: [...this.status.targets],
    };
    return this.status;
  }

  async loopTickAt(generation: number) {
    this.calls.push({ name: "loopTickAt", value: generation });
    return this.loopTick();
  }

  async loopStatus() {
    this.calls.push({ name: "loopStatus" });
    return this.status;
  }

  async runtimeClose(_generation = 0) {
    this.calls.push({ name: "runtimeClose" });
    this.status = null;
    this.previewStatus = null;
  }

  async loopClose(_generation = 0) {
    this.calls.push({ name: "loopClose" });
    this.status = null;
  }

  async previewMachineSelect(virtualMachineId: string) {
    this.calls.push({ name: "previewMachineSelect", value: virtualMachineId });
    if (!this.machines.some((machine) => machine.id === virtualMachineId))
      throw new Error("unknown virtual machine");
  }

  async previewLoopOpen(virtualMachineId: string) {
    this.calls.push({ name: "previewLoopOpen", value: virtualMachineId });
    if (!this.machines.some((machine) => machine.id === virtualMachineId))
      throw new Error(`unknown virtual machine: ${virtualMachineId}`);
    this.previewStatus = {
      runtimeState: "running",
      hasExecution: true,
      activeBody: { virtualMachineId, physicalMachineId: null },
      simulation: null,
      readSources: [],
      runtimeGeneration: 1,
      executionGeneration: 1,
      configRevision: 1,
      phase: "ready",
      session: 1,
      tick: 0,
      torque: false,
      positions: Array(15).fill(0),
      targets: Array(15).fill(0),
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
      sensorFailures: 0,
      lastError: null,
      jointRangeStates: [],
    };
    this.previewCommandEncoding = null;
    return {
      ...this.previewStatus,
      positions: [...this.previewStatus.positions],
      targets: [...this.previewStatus.targets],
    };
  }

  async previewLoopPolicy(id: string) {
    this.calls.push({ name: "previewLoopPolicy", value: id });
    if (!this.previewStatus) throw new Error("preview loop is closed");
    const policy = this.policyEntries().find((entry) => entry.id === id);
    if (!policy?.cached) throw new Error("请先下载这个官方策略");
    this.previewCommandEncoding = policy.command?.encoding ?? null;
    this.previewStatus = { ...this.previewStatus, torque: true, lastError: null };
    return {
      ...this.previewStatus,
      positions: [...this.previewStatus.positions],
      targets: [...this.previewStatus.targets],
    };
  }

  async previewLoopPolicyAt(generation: number, id: string) {
    this.calls.push({ name: "previewLoopPolicyAt", value: { generation, id } });
    return this.previewLoopPolicy(id);
  }

  async previewLoopCommand(input: PreviewInput) {
    this.calls.push({ name: "previewLoopCommand", value: input });
    if (!this.previewStatus) throw new Error("preview loop is closed");
    this.previewInput = input;
    return { ...this.previewStatus };
  }

  async previewLoopCommandAt(generation: number, input: PreviewInput) {
    this.calls.push({ name: "previewLoopCommandAt", value: { generation, input } });
    return this.previewLoopCommand(input);
  }

  async previewLoopResetWorld() {
    this.calls.push({ name: "previewLoopResetWorld" });
    if (!this.previewStatus) throw new Error("preview loop is closed");
    this.previewStatus = {
      ...this.previewStatus,
      world: { position: [0, 0], yaw: 0, distanceM: 0 },
    };
    return { ...this.previewStatus };
  }

  async previewLoopResetWorldAt(generation: number) {
    this.calls.push({ name: "previewLoopResetWorldAt", value: generation });
    return this.previewLoopResetWorld();
  }

  async previewLoopTick() {
    this.calls.push({ name: "previewLoopTick" });
    if (!this.previewStatus) throw new Error("preview loop is closed");
    const tick = this.previewStatus.tick + 1;
    const targets = this.previewStatus.torque
      ? Array.from({ length: 15 }, (_, index) => Math.sin(tick / 12 + index * 0.18) * 0.08)
      : this.previewStatus.targets;
    const movesWorld = !["phase", "posture_flag"].includes(this.previewCommandEncoding ?? "");
    this.previewStatus = {
      ...this.previewStatus,
      phase:
        this.previewStatus.torque && this.previewStatus.phase === "ready"
          ? "driving"
          : this.previewStatus.phase,
      tick,
      targets: [...targets],
      positions: [...targets],
      world:
        this.previewStatus.torque && movesWorld
          ? {
              position: [
                (this.previewStatus.world.position[0] ?? 0) +
                  (this.previewInput.twist[0] ?? 0) * 0.02,
                (this.previewStatus.world.position[1] ?? 0) +
                  (this.previewInput.twist[1] ?? 0) * 0.02,
              ],
              yaw: (this.previewStatus.world.yaw ?? 0) + (this.previewInput.twist[2] ?? 0) * 0.02,
              distanceM:
                (this.previewStatus.world.distanceM ?? 0) +
                Math.hypot(this.previewInput.twist[0] ?? 0, this.previewInput.twist[1] ?? 0) * 0.02,
            }
          : this.previewStatus.world,
    };
    return {
      ...this.previewStatus,
      positions: [...this.previewStatus.positions],
      targets: [...this.previewStatus.targets],
    };
  }

  async previewLoopTickAt(generation: number) {
    this.calls.push({ name: "previewLoopTickAt", value: generation });
    return this.previewLoopTick();
  }

  async previewLoopClose(_generation = 0) {
    this.calls.push({ name: "previewLoopClose" });
    this.previewStatus = null;
  }

  async previewLoopCloseAt(generation: number) {
    this.calls.push({ name: "previewLoopCloseAt", value: generation });
    return this.previewLoopClose(generation);
  }
}
