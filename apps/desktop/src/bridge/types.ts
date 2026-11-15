export * from "../../generated/types";

import type {
  ActionRecord,
  LoopAction,
  LoopCommandInput,
  PreviewInput,
  LoopStatus,
  PhysicalMachine,
  JointRange,
  DetectedPhysicalMachine,
  PhysicalMachineDetectInput,
  PhysicalMachineSaveInput,
  PhysicalMachineProbeInput,
  PhysicalMachineReconnectInput,
  PhysicalMachineInput,
  PhysicalServoRegister,
  PolicyEntry,
  SerialPortCandidate,
  SceneFrame,
  SceneInfo,
  ServoCatalogEntry,
  VirtualMachine,
} from "../../generated/types";

export interface DesktopBackend {
  servoCatalog(): Promise<ServoCatalogEntry[]>;
  officialJointSlots(): Promise<string[]>;
  officialJointRanges(): Promise<JointRange[]>;
  virtualMachines(): Promise<VirtualMachine[]>;
  physicalMachines(): Promise<PhysicalMachine[]>;
  autoReconnectEnabled(): Promise<boolean>;
  setAutoReconnectEnabled(enabled: boolean): Promise<boolean>;
  physicalPorts(): Promise<SerialPortCandidate[]>;
  physicalBaudChoices(): Promise<number[]>;
  probePhysicalMachine(input: PhysicalMachineProbeInput): Promise<DetectedPhysicalMachine>;
  detectPhysicalMachine(input: PhysicalMachineDetectInput): Promise<DetectedPhysicalMachine>;
  savePhysicalMachine(input: PhysicalMachineSaveInput): Promise<PhysicalMachine>;
  disconnectPhysicalMachine(id: string): Promise<PhysicalMachine>;
  reconnectPhysicalMachine(input: PhysicalMachineReconnectInput): Promise<PhysicalMachine>;
  autoReconnectPhysicalMachine(id: string, baud: number): Promise<PhysicalMachine>;
  createPhysicalMachine(input: PhysicalMachineInput): Promise<PhysicalMachine>;
  connectPhysicalMachine(id: string): Promise<PhysicalMachine>;
  mountPhysicalMachine(id: string, virtualMachineId: string): Promise<PhysicalMachine>;
  deletePhysicalMachine(id: string): Promise<void>;
  renamePhysicalMachine(id: string, name: string): Promise<PhysicalMachine>;
  bindPhysicalMachine(input: {
    physicalMachineId: string;
    virtualMachineId?: string | null;
    slotId: string;
    physicalId: number | null;
  }): Promise<PhysicalMachine>;
  bindVirtualMachine(input: {
    virtualMachineId: string;
    slotId: string;
    physicalId: number | null;
  }): Promise<VirtualMachine>;
  validateVirtualMachineBindings(id: string): Promise<VirtualMachine>;
  validatePhysicalMachineBindings(id: string): Promise<PhysicalMachine>;
  setPhysicalMachinePoseZero(
    id: string,
    virtualMachineId?: string | null,
  ): Promise<PhysicalMachine>;
  physicalMachinePose(id: string, virtualMachineId?: string | null): Promise<number[]>;
  setPhysicalMachineTorque(
    id: string,
    enabled: boolean,
    virtualMachineId?: string | null,
  ): Promise<PhysicalMachine>;
  writePhysicalMachinePosition(input: {
    physicalMachineId: string;
    servoId: number;
    positionRad: number;
  }): Promise<PhysicalMachine>;
  scanPhysicalMachine(id: string): Promise<PhysicalMachine>;
  physicalMachineRegisters(id: string): Promise<PhysicalServoRegister[]>;
  readPhysicalMachineRegister(
    physicalMachineId: string,
    servoId: number,
    name: string,
  ): Promise<PhysicalServoRegister>;
  writePhysicalMachineRegister(input: {
    physicalMachineId: string;
    servoId: number;
    name: string;
    value: number;
  }): Promise<PhysicalServoRegister>;
  policyList(): Promise<PolicyEntry[]>;
  policyDownload(id: string): Promise<PolicyEntry>;
  policySelect(id: string): Promise<PolicyEntry>;
  policyOpenFolder(): Promise<void>;
  openLogs(): Promise<void>;
  createVirtualMachine(
    input: Pick<VirtualMachine, "name" | "bodyId" | "servoModel" | "simulation">,
  ): Promise<VirtualMachine>;
  setStartupVirtualMachine(id: string): Promise<void>;
  setVirtualMachineWorldPosition(
    id: string,
    position: [number, number, number],
  ): Promise<VirtualMachine>;
  deleteVirtualMachine(id: string): Promise<void>;
  renameVirtualMachine(id: string, name: string): Promise<VirtualMachine>;
  actions(): Promise<ActionRecord[]>;
  createAction(input: Pick<ActionRecord, "name" | "kind">): Promise<ActionRecord>;
  deleteAction(id: string): Promise<void>;
  loopOpen(input: {
    virtualMachineId: string;
    servoIds: number[];
    physicalMachineId?: string;
    policyId?: string;
  }): Promise<LoopStatus>;
  loopCommand(targets: number[]): Promise<void>;
  loopPolicyCommand(input: LoopCommandInput): Promise<void>;
  loopAction(action: LoopAction): Promise<LoopStatus>;
  loopTick(): Promise<LoopStatus>;
  loopStatus(): Promise<LoopStatus | null>;
  loopClose(): Promise<void>;
  previewLoopOpen(virtualMachineId: string): Promise<LoopStatus>;
  previewLoopPolicy(policyId: string): Promise<LoopStatus>;
  previewLoopCommand(input: PreviewInput): Promise<LoopStatus>;
  previewLoopResetWorld(): Promise<LoopStatus>;
  previewLoopTick(): Promise<LoopStatus>;
  previewLoopClose(): Promise<void>;
  officialSceneModel(): Promise<SceneInfo>;
  officialSceneFrame(positions: number[]): Promise<SceneFrame>;
}
