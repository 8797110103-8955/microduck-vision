export * from "../../generated/types";

import type {
  ActionRecord,
  LoopAction,
  LoopStatus,
  PhysicalMachine,
  DetectedPhysicalMachine,
  PhysicalMachineDetectInput,
  PhysicalMachineSaveInput,
  PhysicalMachineProbeInput,
  PhysicalMachineReconnectInput,
  PhysicalMachineInput,
  PolicyEntry,
  SerialPortCandidate,
  SceneFrame,
  SceneInfo,
  ServoCatalogEntry,
  VirtualMachine,
} from "../../generated/types";

export interface DesktopBackend {
  servoCatalog(): Promise<ServoCatalogEntry[]>;
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
  scanPhysicalMachine(id: string): Promise<PhysicalMachine>;
  policyList(): Promise<PolicyEntry[]>;
  policyDownload(id: string): Promise<PolicyEntry>;
  policySelect(id: string): Promise<PolicyEntry>;
  createVirtualMachine(
    input: Pick<VirtualMachine, "name" | "bodyId" | "servoModel" | "simulation">,
  ): Promise<VirtualMachine>;
  setStartupVirtualMachine(id: string): Promise<void>;
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
  loopAction(action: LoopAction): Promise<LoopStatus>;
  loopTick(): Promise<LoopStatus>;
  loopStatus(): Promise<LoopStatus | null>;
  loopClose(): Promise<void>;
  previewLoopOpen(virtualMachineId: string): Promise<LoopStatus>;
  previewLoopPolicy(policyId: string): Promise<LoopStatus>;
  previewLoopTick(): Promise<LoopStatus>;
  previewLoopClose(): Promise<void>;
  officialSceneModel(): Promise<SceneInfo>;
  officialSceneFrame(positions: number[]): Promise<SceneFrame>;
}
