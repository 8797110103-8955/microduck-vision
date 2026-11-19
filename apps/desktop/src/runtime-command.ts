import { runtimeCommand } from "../generated/types";

let sequence = 0;

/** One monotonic request sequence shared by every page using the same Rust runtime. */
export function sendRuntimeCommand(
  generation: Parameters<typeof runtimeCommand>[0],
  executionGeneration: Parameters<typeof runtimeCommand>[1],
  request: Parameters<typeof runtimeCommand>[3],
) {
  return runtimeCommand(generation, executionGeneration, ++sequence, request);
}
