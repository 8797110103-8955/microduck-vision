import { flushPromises } from "@vue/test-utils";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import * as THREE from "three";
import type { SceneFrame, SimulationState } from "../../generated/types";
import { DuckViewer } from "../../src/components/duck-stage/viewer";

let renderedScene: THREE.Scene | undefined;
let renderedCamera: THREE.PerspectiveCamera | undefined;
let renderFrame: (() => void) | undefined;

vi.mock("@tauri-apps/api/core", () => ({ convertFileSrc: (path: string) => path }));

vi.mock("three", async (importOriginal) => {
  const actual = await importOriginal<typeof import("three")>();
  return {
    ...actual,
    WebGLRenderer: class {
      domElement = document.createElement("canvas");
      setPixelRatio() {}
      setSize() {}
      dispose() {}
      render(scene: THREE.Scene, camera: THREE.PerspectiveCamera) {
        renderedScene = scene;
        renderedCamera = camera;
      }
    },
  };
});
vi.mock("three/examples/jsm/controls/OrbitControls.js", () => ({
  OrbitControls: class {
    constructor(private camera: THREE.PerspectiveCamera) {}
    target = new THREE.Vector3();
    mouseButtons = {};
    update() {
      this.camera.lookAt(this.target);
      this.camera.updateMatrixWorld();
    }
    addEventListener() {}
    removeEventListener() {}
    dispose() {}
  },
}));
vi.mock("three/examples/jsm/loaders/STLLoader.js", () => ({
  STLLoader: class {
    async loadAsync() {
      return new THREE.BoxGeometry(0.02, 0.02, 0.02);
    }
  },
}));

const staticFrame: SceneFrame = {
  bodies: [
    [0, 0, 0, 1, 0, 0, 0],
    [0.2, 0, 0.1, 1, 0, 0, 0],
  ],
  markers: [],
};
function simulation(x: number, y: number, yaw: number): SimulationState {
  const w = Math.cos(yaw / 2);
  const z = Math.sin(yaw / 2);
  return {
    physicsStep: 4,
    time: 0.02,
    trunkPosition: [x, y, 0.125],
    trunkQuaternion: [w, 0, 0, z],
    frame: {
      bodies: [
        [x, y, 0.125, w, 0, 0, z],
        [x + 0.2 * Math.cos(yaw), y + 0.2 * Math.sin(yaw), 0.225, w, 0, 0, z],
      ],
      markers: [],
    },
    motorPositions: [],
    passivePositions: [],
    baseVelocity: [0, 0, 0, 0, 0, 0],
    contacts: [],
    motorTorques: [],
    frictionBudgets: [],
    effectiveVoltage: 5,
  };
}

const roots = () => renderedScene!.children.filter((child) => child instanceof THREE.Group);
const bodyPosition = (duck: number, body: number) => {
  renderedScene!.updateMatrixWorld(true);
  return roots()[duck].children[body].getWorldPosition(new THREE.Vector3());
};

describe("duck world coordinates", () => {
  let viewer: DuckViewer;
  beforeEach(() => {
    localStorage.clear();
    vi.stubGlobal("requestAnimationFrame", (callback: () => void) => {
      renderFrame = callback;
      return 1;
    });
    vi.stubGlobal("cancelAnimationFrame", () => {});
    viewer = new DuckViewer(document.createElement("div"), {
      meshDir: "",
      defaultFrame: staticFrame,
      model: {
        trunkHeight: 0.125,
        bodies: [
          { name: "trunk", geoms: [] },
          { name: "head", geoms: [] },
        ],
        joints: [],
        headCamera: null,
      },
    });
  });
  afterEach(() => {
    viewer.dispose();
    vi.unstubAllGlobals();
  });

  function loadFootMesh() {
    viewer.dispose();
    const host = document.createElement("div");
    Object.defineProperties(host, { clientWidth: { value: 1200 }, clientHeight: { value: 800 } });
    // A saved pan should not override startup's focus on the selected duck.
    localStorage.setItem(
      "microduck-vision.viewer.view",
      JSON.stringify({ camera: [9, 6, 9], target: [8, 5, 8] }),
    );
    viewer = new DuckViewer(host, {
      meshDir: "",
      defaultFrame: {
        bodies: [
          [0, 0, 0, 1, 0, 0, 0],
          [0.2, 0, -0.1, 1, 0, 0, 0],
        ],
        markers: [],
      },
      model: {
        trunkHeight: 0.125,
        joints: [],
        headCamera: null,
        bodies: [
          { name: "trunk", geoms: [] },
          {
            name: "foot",
            geoms: [{ mesh: "foot.stl", pos: [0, 0, 0], quat: [1, 0, 0, 0], rgba: [1, 1, 1, 1] }],
          },
        ],
      },
    });
  }

  it("grounds default feet while preserving explicit scene height and native physics height", async () => {
    loadFootMesh();
    viewer.setGhosts(1, 0.25, [
      [0, 0, 0.3],
      [3, 0, 0],
    ]);
    await flushPromises();
    renderFrame?.();
    const selected = new THREE.Box3().setFromObject(roots()[0], true);
    const other = new THREE.Box3().setFromObject(roots()[1], true);
    expect(selected.min.y).toBeCloseTo(0.3, 10);
    expect(other.min.y).toBeCloseTo(0, 10);
    viewer.setSimulationFrame(simulation(10, 20, 0.4));
    expect(bodyPosition(0, 0).y).toBeCloseTo(0.425);
  });

  it("initially centers the selected duck in the left half after meshes load", async () => {
    loadFootMesh();
    viewer.setGhosts(1, 0.25, [
      [0, 0, 0],
      [3, 4, 0],
    ]);
    viewer.setSelectedDuck(1);
    viewer.setWorldPose([0.5, 0.6], 0.3);
    await flushPromises();
    renderFrame?.();
    const center = new THREE.Box3().setFromObject(roots()[1]).getCenter(new THREE.Vector3());
    const projected = center.clone().project(renderedCamera!);
    expect(projected.x).toBeCloseTo(-0.5, 10);
    expect(projected.y).toBeCloseTo(0, 10);
    expect(projected.z).toBeLessThan(1);
  });

  it("articulates all ducks at construction before any telemetry arrives", () => {
    viewer.setGhosts(1, 0.25, [
      [0, 0, 0],
      [3, 0, 0],
    ]);
    expect(bodyPosition(0, 1).distanceTo(bodyPosition(0, 0))).toBeCloseTo(Math.hypot(0.2, 0.1));
    expect(bodyPosition(1, 1).distanceTo(bodyPosition(1, 0))).toBeCloseTo(Math.hypot(0.2, 0.1));
  });

  it("keeps idle ducks articulated when the first live frame comes from MuJoCo", () => {
    viewer.setGhosts(1, 0.25, [
      [0, 0, 0],
      [3, 0, 0],
    ]);
    const restTrunk = bodyPosition(1, 0).clone();
    const restHead = bodyPosition(1, 1).clone();
    viewer.setSimulationFrame(simulation(20, 30, 0.4));
    expect(bodyPosition(1, 0).distanceTo(restTrunk)).toBeLessThan(1e-12);
    expect(bodyPosition(1, 1).distanceTo(restHead)).toBeLessThan(1e-12);
    viewer.setSelectedDuck(1);
    viewer.setSimulationFrame(simulation(-5, 6, -0.8));
    expect(bodyPosition(0, 1).distanceTo(bodyPosition(0, 0))).toBeCloseTo(Math.hypot(0.2, 0.1));
    viewer.setGhosts(2, 0.25, [
      [0, 0, 0],
      [3, 0, 0],
      [6, 0, 0],
    ]);
    expect(bodyPosition(2, 1).distanceTo(bodyPosition(2, 0))).toBeCloseTo(Math.hypot(0.2, 0.1));
  });

  it("keeps the same placement and heading through idle, simulation and sensor views", () => {
    viewer.setGhosts(0, 0.25, [[1, 2, 0.3]]);
    viewer.setWorldPose([1.2, 2.4], 0.6);
    const trunk = bodyPosition(0, 0);
    const head = bodyPosition(0, 1);
    expect(trunk.x).toBeCloseTo(2.2);
    expect(trunk.y).toBeCloseTo(0.425);
    expect(trunk.z).toBeCloseTo(-4.4);

    viewer.setSimulationFrame(simulation(10, -5, -0.7));
    expect(bodyPosition(0, 0).distanceTo(trunk)).toBeLessThan(1e-12);
    expect(bodyPosition(0, 1).distanceTo(head)).toBeLessThan(1e-12);
    // Native settling and yaw drift without a changed world pose cannot move scene placement.
    viewer.setSimulationFrame(simulation(10.1, -5.2, -0.5));
    expect(bodyPosition(0, 0).distanceTo(trunk)).toBeLessThan(1e-12);
    expect(bodyPosition(0, 1).distanceTo(head)).toBeLessThan(1e-12);
    viewer.setFrame(staticFrame);
    expect(bodyPosition(0, 0).distanceTo(trunk)).toBeLessThan(1e-12);
    expect(bodyPosition(0, 1).distanceTo(head)).toBeLessThan(1e-12);
  });

  it("renders a training environment in its own world coordinates", () => {
    viewer.setGhosts(0, 0.25, [[0, 0, 0]]);
    viewer.setWorldPose([2, 3], 0.5);
    const recorded = simulation(0.2, -0.1, 0.4);
    viewer.setSimulationFrame(recorded, true);
    const displayed = bodyPosition(0, 0);
    expect(displayed.x).toBeCloseTo(recorded.trunkPosition[0]!);
    expect(displayed.y).toBeCloseTo(recorded.trunkPosition[2]!);
    expect(displayed.z).toBeCloseTo(-recorded.trunkPosition[1]!);

    viewer.setSimulationFrame(recorded);
    const runtime = bodyPosition(0, 0);
    expect(runtime.x).toBeCloseTo(2);
    expect(runtime.z).toBeCloseTo(-3);
  });

  it("applies policy displacement once and keeps other ducks at their own saved placement", () => {
    viewer.setGhosts(1, 0.25, [
      [0, 0, 0],
      [3, 4, 0],
    ]);
    viewer.setMachineWorldPoses([
      { position: [0, 0], yaw: 0, distanceM: 0 },
      { position: [1, 2], yaw: Math.PI / 2, distanceM: 5 },
    ]);
    viewer.setSimulationFrame(simulation(20, 30, 0.4));
    viewer.setWorldPose([0.7, 0.8], 0.4);
    expect(bodyPosition(0, 0).x).toBeCloseTo(0.7);
    expect(bodyPosition(0, 0).z).toBeCloseTo(-0.8);
    expect(bodyPosition(1, 0).toArray()).toEqual([4, 0.125, -6]);
    expect(bodyPosition(1, 1).x).toBeCloseTo(4);
    expect(bodyPosition(1, 1).z).toBeCloseTo(-6.2);
  });
});
