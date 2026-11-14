import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import { convertFileSrc } from "@tauri-apps/api/core";
import type { SceneFrame, SceneInfo } from "../../bridge/types";

const n = (v: number | null | undefined) => v ?? 0;

type DuckMesh = THREE.Mesh<THREE.BufferGeometry, THREE.MeshStandardMaterial>;

export class DuckViewer {
  private renderer: THREE.WebGLRenderer;
  private scene = new THREE.Scene();
  private camera = new THREE.PerspectiveCamera(35, 1, 0.01, 10);
  private controls: OrbitControls;
  private root = new THREE.Group();
  private bodies: THREE.Group[] = [];
  private meshesByBody: DuckMesh[][] = [];
  private bodyOf = new Map<THREE.Object3D, number>();
  private raycaster = new THREE.Raycaster();
  private frame: SceneFrame | null = null;
  private raf = 0;
  private resize: ResizeObserver;
  private highlighted: number | null = null;
  private dim = false;
  private grid: THREE.GridHelper;
  /** Stand-ins for the other virtual machines: the same model at rest, side by side. */
  private ghosts: THREE.Group[] = [];
  private ghostBodies: THREE.Group[][] = [];
  private duckOf = new Map<THREE.Object3D, number>();
  /** Each ghost mesh and the main-duck mesh whose geometry it borrows. */
  private ghostSources = new Map<DuckMesh, DuckMesh>();

  onRender: (() => void) | null = null;

  constructor(
    private host: HTMLElement,
    private info: SceneInfo,
  ) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    host.appendChild(this.renderer.domElement);

    this.camera.position.set(0.42, 0.22, 0.42);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0.11, 0);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8577, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 1.6);
    sun.position.set(1, 2, 1.5);
    this.scene.add(sun);
    this.grid = new THREE.GridHelper(1, 20, 0xc9c4b5, 0xe4e0d4);
    this.scene.add(this.grid);

    this.root.rotation.x = -Math.PI / 2;
    this.root.position.y = n(info.model.trunkHeight);
    this.scene.add(this.root);
    this.build();

    this.resize = new ResizeObserver(() => this.fit());
    this.resize.observe(host);
    this.fit();
    this.loop();
  }

  private build() {
    const loader = new STLLoader();
    const sep = this.info.meshDir.includes("\\") ? "\\" : "/";
    const cache = new Map<string, Promise<THREE.BufferGeometry>>();
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
        geometry(g.mesh)
          .then((geo) => (mesh.geometry = geo))
          .catch((e) => console.warn("加载网格失败", g.mesh, e));
      }
    });
  }

  setFrame(frame: SceneFrame) {
    this.frame = frame;
    this.pose(this.bodies, frame);
    if (!this.restFrame) {
      this.restFrame = frame;
      this.ghostBodies.forEach((b) => this.pose(b, frame));
    }
  }

  /** The first frame is the default posture the ghosts keep. */
  private restFrame: SceneFrame | null = null;

  private pose(bodies: THREE.Group[], frame: SceneFrame) {
    frame.bodies.forEach((p, i) => {
      const g = bodies[i];
      if (!g) return;
      g.position.set(n(p[0]), n(p[1]), n(p[2]));
      g.quaternion.set(n(p[4]), n(p[5]), n(p[6]), n(p[3]));
    });
  }

  /** `count` more ducks to the right of the main one, `spacing` metres apart; the row stays centred. */
  setGhosts(count: number, spacing = 0.25) {
    if (count === this.ghosts.length) return;
    this.ghosts.forEach((g) => this.scene.remove(g));
    this.ghosts = [];
    this.ghostBodies = [];
    this.duckOf.clear();
    this.ghostSources.clear();
    this.root.position.x = (-spacing * count) / 2;
    this.root.updateMatrixWorld();
    for (let k = 1; k <= count; k++) {
      const root = new THREE.Group();
      root.rotation.copy(this.root.rotation);
      root.position.copy(this.root.position);
      root.position.x += spacing * k;
      const bodies = this.bodies.map((_, i) => {
        const g = new THREE.Group();
        for (const m of this.meshesByBody[i]) {
          const mat = m.material.clone();
          mat.opacity = 0.55;
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
    if (body === this.highlighted) return;
    const set = (b: number | null, on: boolean) =>
      b !== null &&
      this.meshesByBody[b]?.forEach((m) => {
        const mat = m.material;
        mat.emissive.set(on ? 0xf5b700 : 0x000000);
        mat.emissiveIntensity = on ? 0.9 : 0;
      });
    set(this.highlighted, false);
    set(body, true);
    this.highlighted = body;
  }

  pick(clientX: number, clientY: number): number | null {
    this.raycaster.setFromCamera(this.ndc(clientX, clientY), this.camera);
    const hit = this.raycaster.intersectObjects(this.meshesByBody.flat(), false)[0];
    return hit ? (this.bodyOf.get(hit.object) ?? null) : null;
  }

  markerScreen(i: number): { x: number; y: number } | null {
    const m = this.frame?.markers[i];
    if (!m) return null;
    const v = new THREE.Vector3(n(m[0]), n(m[1]), n(m[2]))
      .applyMatrix4(this.root.matrixWorld)
      .project(this.camera);
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
  }

  private loop = () => {
    this.raf = requestAnimationFrame(this.loop);
    // Meshes load asynchronously; ghosts pick up the main duck's geometry once it arrives.
    for (const [c, src] of this.ghostSources)
      if (c.geometry !== src.geometry) c.geometry = src.geometry;
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
    this.onRender?.();
  };

  dispose() {
    cancelAnimationFrame(this.raf);
    this.resize.disconnect();
    this.controls.dispose();
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
