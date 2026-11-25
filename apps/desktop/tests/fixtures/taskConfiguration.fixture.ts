import type * as commands from "../../generated/types";

export function taskConfiguration(
  taskId: string,
): Awaited<ReturnType<typeof commands.trainingTaskConfiguration>> {
  return {
    taskId,
    runner: "fixture.Runner",
    algorithm: "PPO",
    maxIterations: 100,
    training: {
      numEnvs: 64,
      controlHz: 50,
      episodeLengthS: 20,
      terrain: "plane",
      json: '{"mode":"training"}',
    },
    play: {
      numEnvs: 1,
      controlHz: 50,
      episodeLengthS: 40,
      terrain: "plane",
      json: '{"mode":"play"}',
    },
    rlJson: '{"algorithm":"PPO"}',
  };
}
