import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { convertFileSrc } from "@tauri-apps/api/core";
import type { SceneFrame, SceneInfo, SimulationState, WorldState } from "../../../generated/types";

const n = (v: number | null | undefined) => v ?? 0;
const isRecord = (value: unknown): value is Record<string, unknown> =>
  !!value && typeof value === "object" && !Array.isArray(value);
const finiteVector = (value: unknown): value is [number, number, number] =>
  Array.isArray(value) &&
  value.length === 3 &&
  value.every((entry) => typeof entry === "number" && Number.isFinite(entry));

type DuckMesh = THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;
type ViewState = {
  camera: THREE.Vector3;
  target: THREE.Vector3;
};

export class DuckViewer {
  private static readonly viewStorageKey = "microduck-vision.viewer.view";
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(35, 1, 0.01, 1000);
  private controls: OrbitControls;
  private root = new THREE.Group();
  private bodies: THREE.Group[] = [];
  private meshesByBody: DuckMesh[][] = [];
  private bodyOf = new Map<THREE.Object3D, number>();
  private raycaster = new THREE.Raycaster();
  private frame: SceneFrame | null = null;
  private raf = 0;
  private resize: ResizeObserver;
  private highlightedBodies = new Set<number>();
  private dim = false;
  private grid: THREE.GridHelper;
  /** Stand-ins for the other virtual machines: the same model at rest, side by side. */
  private ghosts: THREE.Group[] = [];
  private ghostBodies: THREE.Group[][] = [];
  private duckOf = new Map<THREE.Object3D, number>();
  /** Each ghost mesh and the main-duck mesh whose geometry it borrows. */
  private ghostSources = new Map<DuckMesh, DuckMesh>();
  private worldPose: [number, number, number] = [0, 0, 0];
  private simulationFrame = false;
  private preserveSimulationWorld = false;
  private simulationPose: [number, number, number] = [0, 0, 0];
  /** Scene offsets share the simulation's right-handed Z-up basis. */
  private configuredWorldPositions: [number, number, number][] = [];
  private selectedDuckIndex = 0;
  private machineWorldPoses: (WorldState | undefined)[] = [];
  private static readonly ghostOpacity = 0.55;
  private baseHeight: number;
  /** Convert the vendor/MuJoCo Z-up frame into the Three.js Y-up frame. */
  private readonly mujocoToThree = new THREE.Quaternion().setFromEuler(
    new THREE.Euler(-Math.PI / 2, 0, 0),
  );
  private readonly mujocoUp = new THREE.Vector3(0, 0, 1);
  private readonly yawRotation = new THREE.Quaternion();
  private viewRestored = false;
  private defaultFocused = false;
  private meshesReady = false;
  private disposed = false;
  private previousFocusedView: ViewState | null = null;

  onRender: (() => void) | null = null;

  constructor(
    private host: HTMLElement,
    private info: SceneInfo,
  ) {
    this.baseHeight = n(info.model.trunkHeight);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    host.appendChild(this.renderer.domElement);

    this.camera.position.set(0.42, 0.22, 0.42);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.11, 0);
    this.controls.enableDamping = true;
    this.controls.enableZoom = true;
    this.controls.enablePan = true;
    this.controls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.DOLLY,
      RIGHT: THREE.MOUSE.PAN,
    };
    this.renderer.domElement.style.touchAction = "none";
    this.renderer.domElement.addEventListener("contextmenu", this.preventContextMenu);
    this.viewRestored = this.restoreView();
    this.controls.addEventListener("end", this.saveView);

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8577, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1, 2, 1.5);
    this.scene.add(sun);
    this.grid = new THREE.GridHelper(1000, 1000, 0xc9c4b5, 0xe4e0d4);
    this.scene.add(this.grid);

    this.setRootYaw(this.root, 0);
    this.root.position.y = this.baseHeight;
    this.scene.add(this.root);
    this.build();
    // Every machine needs articulation before its first render, including machines that
    // never receive telemetry. A live frame must never become another duck's rest pose.
    this.setFrame(info.defaultFrame);

    this.resize = new ResizeObserver(() => this.fit());
    this.resize.observe(host);
    this.fit();
    this.loop();
  }

  private build() {
    const loader = new STLLoader();
    const sep = this.info.meshDir.includes("\\") ? "\\" : "/";
    const cache = new Map<string, Promise<THREE.BufferGeometry>>();
    const pending: Promise<unknown>[] = [];
    const geometry = (file: string) => {
      if (!cache.has(file))
        cache.set(file, loader.loadAsync(convertFileSrc(this.info.meshDir + sep + file)));
      return cache.get(file)!;
    };
    this.info.model.bodies.forEach((b, i) => {
      const group = new THREE.Group();
      this.root.add(group);
      this.bodies.push(group);
      this.meshesByBody.push([]);
      for (const g of b.geoms) {
        const material = new THREE.MeshStandardMaterial({
          color: new THREE.Color(n(g.rgba[0]), n(g.rgba[1]), n(g.rgba[2])),
          roughness: 0.6,
          metalness: 0.05,
          transparent: true,
        });
        const mesh: DuckMesh = new THREE.Mesh(new THREE.BufferGeometry(), material);
        this.bodyOf.set(mesh, i);
        mesh.position.set(n(g.pos[0]), n(g.pos[1]), n(g.pos[2]));
        mesh.quaternion.set(n(g.quat[1]), n(g.quat[2]), n(g.quat[3]), n(g.quat[0]));
        group.add(mesh);
        this.meshesByBody[i].push(mesh);
        pending.push(
          geometry(g.mesh)
            .then((geo) => (mesh.geometry = geo))
            .catch((e) => console.warn("加载网格失败", g.mesh, e)),
        );
      }
    });
    void Promise.allSettled(pending).then(() => {
      if (this.disposed) return;
      this.syncGhostGeometry();
      this.groundDefaultPose();
      this.meshesReady = true;
      this.fit();
    });
  }

  /** Derive static placement from the official rest meshes, independently of live physics. */
  private groundDefaultPose() {
    const basis = new THREE.Matrix4().makeRotationFromQuaternion(this.mujocoToThree);
    const body = new THREE.Matrix4();
    const transform = new THREE.Matrix4();
    const point = new THREE.Vector3();
    const position = new THREE.Vector3();
    const rotation = new THREE.Quaternion();
    const scale = new THREE.Vector3(1, 1, 1);
    let lowest = Infinity;
    this.info.defaultFrame.bodies.forEach((pose, index) => {
      position.set(n(pose[0]), n(pose[1]), n(pose[2]));
      rotation.set(n(pose[4]), n(pose[5]), n(pose[6]), n(pose[3]));
      body.compose(position, rotation, scale);
      for (const mesh of this.meshesByBody[index] ?? []) {
        mesh.updateMatrix();
        transform.copy(basis).multiply(body).multiply(mesh.matrix);
        const vertices = mesh.geometry.getAttribute("position");
        if (!vertices) continue;
        for (let vertex = 0; vertex < vertices.count; vertex++) {
          point.fromBufferAttribute(vertices, vertex).applyMatrix4(transform);
          lowest = Math.min(lowest, point.y);
        }
      }
    });
    if (Number.isFinite(lowest)) {
      this.baseHeight = -lowest;
      this.applyMachineWorldPositions();
    }
  }

  setFrame(frame: SceneFrame) {
    this.simulationFrame = false;
    this.applyMachineWorldPositions();
    this.frame = frame;
    this.pose(this.bodiesForDuck(this.selectedDuckIndex), frame);
    if (!this.restFrame) {
      this.restFrame = frame;
      this.ghostBodies.forEach((b) => this.pose(b, frame));
    }
  }

  /** Keep native articulation, height and roll/pitch. Map its temporary planar origin
   * onto the duck's authoritative scene placement exactly once. */
  setSimulationFrame(
    simulation: Pick<SimulationState, "frame" | "trunkPosition" | "trunkQuaternion">,
    preserveWorld = false,
  ) {
    const frame = simulation.frame;
    const [w, x, y, z] = simulation.trunkQuaternion.map(n);
    this.simulationPose = [
      n(simulation.trunkPosition[0]),
      n(simulation.trunkPosition[1]),
      Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    ];
    this.preserveSimulationWorld = preserveWorld;
    this.simulationFrame = true;
    this.applyMachineWorldPositions();
    this.frame = frame;
    this.pose(this.bodiesForDuck(this.selectedDuckIndex), frame);
  }

  setSelectedDuck(index: number) {
    const next = Math.max(0, Math.min(index, this.ghosts.length));
    if (next === this.selectedDuckIndex) return;
    const previous = this.selectedDuckIndex;
    this.selectedDuckIndex = next;
    if (this.restFrame) {
      this.pose(this.bodiesForDuck(previous), this.restFrame);
      this.pose(this.bodiesForDuck(next), this.restFrame);
    }
    this.applyMachineWorldPositions();
    this.updateDuckOpacity();
    this.highlightedBodies.forEach((body) => {
      this.setBodyHighlight(previous, body, false);
      this.setBodyHighlight(next, body, true);
    });
  }

  focusCurrentDuck() {
    if (!this.previousFocusedView) {
      this.previousFocusedView = {
        camera: this.camera.position.clone(),
        target: this.controls.target.clone(),
      };
    }
    this.focusDuck(true, true);
  }

  restorePreviousView() {
    if (!this.previousFocusedView) return;
    this.camera.position.copy(this.previousFocusedView.camera);
    this.controls.target.copy(this.previousFocusedView.target);
    this.controls.update();
    this.previousFocusedView = null;
    this.saveView();
  }

  private focusDuck(adjustDistance: boolean, alignLeftHalf = false) {
    const root = this.rootForDuck(this.selectedDuckIndex);
    root.updateMatrixWorld(true);
    const bounds = new THREE.Box3().setFromObject(root);
    if (bounds.isEmpty()) return;
    const center = bounds.getCenter(new THREE.Vector3());
    const direction = this.camera.position.clone().sub(this.controls.target);
    if (direction.lengthSq() < 1e-8) direction.set(1, 0.55, 1);
    direction.normalize();
    if (adjustDistance) {
      const size = bounds.getSize(new THREE.Vector3());
      const halfFov = THREE.MathUtils.degToRad(this.camera.fov / 2);
      const verticalDistance = size.y / (2 * Math.tan(halfFov));
      const availableAspect = this.camera.aspect * (alignLeftHalf ? 0.5 : 1);
      const horizontalDistance = size.x / (2 * Math.tan(halfFov) * availableAspect);
      const distance = Math.max(verticalDistance, horizontalDistance, size.z / 2) * 1.7;
      this.camera.position.copy(center).add(direction.multiplyScalar(Math.max(distance, 0.2)));
    }
    this.controls.target.copy(center);
    if (alignLeftHalf) {
      this.camera.lookAt(center);
      this.camera.updateMatrixWorld();
      const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrixWorld, 0).normalize();
      const distance = this.camera.position.distanceTo(center);
      const halfHeight = distance * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2));
      const halfWidth = halfHeight * this.camera.aspect;
      const shift = right.multiplyScalar(halfWidth * 0.5);
      this.camera.position.add(shift);
      this.controls.target.add(shift);
    }
    this.controls.update();
    this.defaultFocused = true;
    this.saveView();
  }

  setWorldPose(position: [number | null, number | null], yaw: number | null) {
    const x = n(position[0]);
    const y = n(position[1]);
    const angle = n(yaw);
    this.worldPose = [x, y, angle];
    this.applySelectedWorldPose();
  }

  /** Apply a MuJoCo/world yaw without changing the fixed Z-up to Y-up basis. */
  private setRootYaw(root: THREE.Group, yaw: number) {
    root.quaternion
      .copy(this.mujocoToThree)
      .multiply(this.yawRotation.setFromAxisAngle(this.mujocoUp, yaw));
  }

  private applyMachineWorldPositions() {
    const roots = [this.root, ...this.ghosts];
    roots.forEach((root, index) => {
      const [x, y, z] = this.configuredWorldPositions[index] ?? [0, 0, 0];
      const world = this.machineWorldPoses[index];
      root.position.set(x + n(world?.position[0]), this.baseHeight + z, -y - n(world?.position[1]));
      this.setRootYaw(root, n(world?.yaw));
    });
    this.applySelectedWorldPose();
  }

  private applySelectedWorldPose() {
    const root = this.rootForDuck(this.selectedDuckIndex);
    const [x, y, z] = this.configuredWorldPositions[this.selectedDuckIndex] ?? [0, 0, 0];
    if (this.simulationFrame) {
      const [rawX, rawY, rawYaw] = this.simulationPose;
      const [worldX, worldY, worldYaw] = this.preserveSimulationWorld
        ? this.simulationPose
        : this.worldPose;
      const yaw = worldYaw - rawYaw;
      const cos = Math.cos(yaw);
      const sin = Math.sin(yaw);
      // MuJoCo (x,y,z) -> Three.js (x,z,-y), for positions and rotations alike.
      root.position.set(
        x + worldX - (cos * rawX - sin * rawY),
        z,
        -y - worldY + (sin * rawX + cos * rawY),
      );
      this.setRootYaw(root, yaw);
    } else {
      root.position.set(x + this.worldPose[0], this.baseHeight + z, -y - this.worldPose[1]);
      this.setRootYaw(root, this.worldPose[2]);
    }
  }

  setMachineWorldPositions(positions: [number, number, number][]) {
    this.configuredWorldPositions = positions.map(
      (position) => [...position] as [number, number, number],
    );
    this.applyMachineWorldPositions();
  }

  setMachineWorldPoses(worlds: (WorldState | undefined)[]) {
    this.machineWorldPoses = worlds;
    this.applyMachineWorldPositions();
  }

  /** The first frame is the default posture the ghosts keep. */
  private restFrame: SceneFrame | null = null;

  private rootForDuck(index: number) {
    return index === 0 ? this.root : (this.ghosts[index - 1] ?? this.root);
  }

  private bodiesForDuck(index: number) {
    return index === 0 ? this.bodies : (this.ghostBodies[index - 1] ?? this.bodies);
  }

  private meshesForDuckBody(index: number, body: number): DuckMesh[] {
    if (index === 0) return this.meshesByBody[body] ?? [];
    return (this.ghostBodies[index - 1]?.[body]?.children ?? []).filter(
      (child): child is DuckMesh => child instanceof THREE.Mesh,
    );
  }

  private setBodyHighlight(index: number, body: number, on: boolean) {
    this.meshesForDuckBody(index, body).forEach((mesh) => {
      mesh.material.emissive.set(on ? 0xf5b700 : 0x000000);
      mesh.material.emissiveIntensity = on ? 0.9 : 0;
    });
  }

  private pose(bodies: THREE.Group[], frame: SceneFrame) {
    frame.bodies.forEach((p, i) => {
      const g = bodies[i];
      if (!g) return;
      g.position.set(n(p[0]), n(p[1]), n(p[2]));
      g.quaternion.set(n(p[4]), n(p[5]), n(p[6]), n(p[3]));
    });
  }

  /** `count` more ducks to the right of the main one, `spacing` metres apart; the row stays centred. */
  setGhosts(count: number, _spacing = 0.25, positions: [number, number, number][] = []) {
    if (count === this.ghosts.length) {
      this.setMachineWorldPositions(positions);
      this.updateDuckOpacity();
      return;
    }
    this.ghosts.forEach((g) => this.scene.remove(g));
    this.ghosts = [];
    this.ghostBodies = [];
    this.duckOf.clear();
    this.ghostSources.clear();
    this.configuredWorldPositions = positions.map(
      (position) => [...position] as [number, number, number],
    );
    this.applyMachineWorldPositions();
    this.root.updateMatrixWorld();
    for (let k = 1; k <= count; k++) {
      const root = new THREE.Group();
      this.setRootYaw(root, 0);
      const [x, y, z] = this.configuredWorldPositions[k] ?? [0, 0, 0];
      root.position.set(x, this.baseHeight + z, -y);
      const bodies = this.bodies.map((_, i) => {
        const g = new THREE.Group();
        for (const m of this.meshesByBody[i]) {
          const mat = m.material.clone();
          mat.opacity = DuckViewer.ghostOpacity;
          mat.emissiveIntensity = 0;
          const c: DuckMesh = new THREE.Mesh(m.geometry, mat);
          c.position.copy(m.position);
          c.quaternion.copy(m.quaternion);
          this.duckOf.set(c, k);
          this.ghostSources.set(c, m);
          g.add(c);
        }
        root.add(g);
        return g;
      });
      if (this.restFrame) this.pose(bodies, this.restFrame);
      this.scene.add(root);
      this.ghosts.push(root);
      this.ghostBodies.push(bodies);
    }
    this.applyMachineWorldPositions();
    this.updateDuckOpacity();
  }

  private updateDuckOpacity() {
    this.meshesByBody.flat().forEach((mesh) => {
      mesh.material.opacity = this.selectedDuckIndex === 0 ? 1 : DuckViewer.ghostOpacity;
    });
    for (let index = 0; index < this.ghostBodies.length; index++) {
      const duckIndex = index + 1;
      const bodies = this.ghostBodies[index];
      for (let body = 0; body < bodies.length; body++) {
        this.meshesForDuckBody(duckIndex, body).forEach((mesh) => {
          mesh.material.opacity =
            duckIndex === this.selectedDuckIndex ? 1 : DuckViewer.ghostOpacity;
        });
      }
    }
  }

  /** Which duck a click lands on: 0 for the main one, k for the k-th ghost. */
  pickDuck(clientX: number, clientY: number): number | null {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const meshes = [...this.meshesByBody.flat(), ...this.ghostSources.keys()];
    const hit = this.raycaster.intersectObjects(meshes, false)[0];
    if (!hit) return null;
    return this.duckOf.get(hit.object) ?? 0;
  }

  /** Where the k-th duck's label goes on screen: just above its eyes (the head camera). */
  duckScreen(k: number): { x: number; y: number } | null {
    const root = k === 0 ? this.root : this.ghosts[k - 1];
    if (!root) return null;
    const cam = this.info.model.headCamera;
    const head = cam && (k === 0 ? this.bodies : this.ghostBodies[k - 1])?.[cam.body];
    const v = head
      ? head.localToWorld(new THREE.Vector3(n(cam.pos[0]), n(cam.pos[1]), n(cam.pos[2])))
      : new THREE.Vector3(0, 0, 0.16).applyMatrix4(root.matrixWorld);
    v.y += 0.025;
    v.project(this.camera);
    if (v.z > 1) return null;
    const el = this.renderer.domElement;
    return { x: ((v.x + 1) / 2) * el.clientWidth, y: ((1 - v.y) / 2) * el.clientHeight };
  }

  private ndc(clientX: number, clientY: number) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    return new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
  }

  setDim(dim: boolean) {
    if (dim === this.dim) return;
    this.dim = dim;
    this.meshesByBody.flat().forEach((m) => {
      const mat = m.material;
      mat.opacity = dim ? 0.45 : 1;
    });
  }

  highlight(body: number | null) {
    this.highlightBodies(body === null ? [] : [body]);
  }

  highlightBodies(bodies: number[]) {
    const next = [...new Set(bodies)].filter((body) => this.meshesByBody[body]);
    [...this.highlightedBodies]
      .filter((body) => !next.includes(body))
      .forEach((body) => this.setBodyHighlight(this.selectedDuckIndex, body, false));
    next
      .filter((body) => !this.highlightedBodies.has(body))
      .forEach((body) => this.setBodyHighlight(this.selectedDuckIndex, body, true));
    this.highlightedBodies = new Set(next);
  }

  pick(clientX: number, clientY: number): number | null {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const hit = this.raycaster.intersectObjects(this.meshesByBody.flat(), false)[0];
    return hit ? (this.bodyOf.get(hit.object) ?? null) : null;
  }

  markerScreen(i: number): { x: number; y: number } | null {
    const m = this.frame?.markers[i];
    if (!m) return null;
    return this.projectScreenPoint(
      new THREE.Vector3(n(m[0]), n(m[1]), n(m[2])).applyMatrix4(
        this.rootForDuck(this.selectedDuckIndex).matrixWorld,
      ),
    );
  }

  /** Project a fixed point in a body frame for UI-only anchors such as mouth. */
  markerScreenAt(body: number, point: [number, number, number]) {
    const group = this.bodiesForDuck(this.selectedDuckIndex)[body];
    if (!group) return null;
    this.rootForDuck(this.selectedDuckIndex).updateMatrixWorld(true);
    return this.projectScreenPoint(group.localToWorld(new THREE.Vector3(...point)));
  }

  private projectScreenPoint(point: THREE.Vector3): { x: number; y: number } | null {
    const v = point.project(this.camera);
    if (v.z > 1) return null;
    const el = this.renderer.domElement;
    return { x: ((v.x + 1) / 2) * el.clientWidth, y: ((1 - v.y) / 2) * el.clientHeight };
  }

  snapshot(size = 1024, dir: [number, number, number] = [1, 0.35, 0.75], margin = 1.08): string {
    const box = new THREE.Box3().setFromObject(this.root);
    const center = box.getCenter(new THREE.Vector3());
    const radius = box.getBoundingSphere(new THREE.Sphere()).radius * margin;
    const cam = new THREE.PerspectiveCamera(30, 1, 0.01, 10);
    const dist = radius / Math.sin(THREE.MathUtils.degToRad(cam.fov / 2));
    cam.position.copy(center).add(new THREE.Vector3(...dir).normalize().multiplyScalar(dist));
    cam.lookAt(center);

    const r = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      preserveDrawingBuffer: true,
    });
    r.setPixelRatio(1);
    r.setSize(size * 2, size * 2);
    r.setClearColor(0x000000, 0);
    this.grid.visible = false;
    r.render(this.scene, cam);
    this.grid.visible = true;
    const url = trimToSquare(r.domElement, size);
    r.dispose();
    return url;
  }

  private fit() {
    const w = this.host.clientWidth;
    const h = this.host.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    if (this.meshesReady && !this.defaultFocused) this.focusDuck(true, true);
  }

  private restoreView = () => {
    let restored = false;
    try {
      const raw = window.localStorage.getItem(DuckViewer.viewStorageKey);
      if (!raw) return false;
      const saved: unknown = JSON.parse(raw);
      if (!isRecord(saved)) return false;
      if (finiteVector(saved.camera)) {
        this.camera.position.fromArray(saved.camera);
        restored = true;
      }
      if (finiteVector(saved.target)) {
        this.controls.target.fromArray(saved.target);
        restored = true;
      }
      this.controls.update();
    } catch {
      // A stale or unavailable browser cache should not prevent the viewport from loading.
    }
    return restored;
  };

  private saveView = () => {
    try {
      window.localStorage.setItem(
        DuckViewer.viewStorageKey,
        JSON.stringify({
          camera: this.camera.position.toArray(),
          target: this.controls.target.toArray(),
        }),
      );
    } catch {
      // Browser storage is optional in embedded webviews.
    }
  };

  private preventContextMenu = (event: MouseEvent) => {
    event.preventDefault();
  };

  private syncGhostGeometry() {
    for (const [clone, source] of this.ghostSources)
      if (clone.geometry !== source.geometry) clone.geometry = source.geometry;
  }

  private loop = () => {
    this.raf = requestAnimationFrame(this.loop);
    // Meshes load asynchronously; ghosts pick up the main duck's geometry once it arrives.
    this.syncGhostGeometry();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.onRender?.();
  };

  dispose() {
    this.disposed = true;
    this.saveView();
    cancelAnimationFrame(this.raf);
    this.resize.disconnect();
    this.controls.removeEventListener("end", this.saveView);
    this.controls.dispose();
    this.renderer.domElement.removeEventListener("contextmenu", this.preventContextMenu);
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

function trimToSquare(src: HTMLCanvasElement, size: number): string {
  const tmp = document.createElement("canvas");
  tmp.width = src.width;
  tmp.height = src.height;
  const ctx = tmp.getContext("2d")!;
  ctx.drawImage(src, 0, 0);
  const { data, width, height } = ctx.getImageData(0, 0, tmp.width, tmp.height);
  let [x0, y0, x1, y1] = [width, height, 0, 0];
  for (let y = 0; y < height; y++)
    for (let x = 0; x < width; x++)
      if (data[(y * width + x) * 4 + 3] > 8) {
        x0 = Math.min(x0, x);
        x1 = Math.max(x1, x);
        y0 = Math.min(y0, y);
        y1 = Math.max(y1, y);
      }
  if (x1 < x0) return src.toDataURL("image/png");
  const w = x1 - x0 + 1;
  const h = y1 - y0 + 1;
  const out = document.createElement("canvas");
  out.width = out.height = size;
  const scale = (size * 0.92) / Math.max(w, h);
  const o = out.getContext("2d")!;
  o.imageSmoothingQuality = "high";
  o.drawImage(
    tmp,
    x0,
    y0,
    w,
    h,
    (size - w * scale) / 2,
    (size - h * scale) / 2,
    w * scale,
    h * scale,
  );
  return out.toDataURL("image/png");
}
