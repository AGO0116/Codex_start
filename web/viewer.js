import * as THREE from "https://cdn.jsdelivr.net/npm/three@0.165.0/build/three.module.js";
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.165.0/examples/jsm/controls/OrbitControls.js";

const defaultConfig = {
  map: { width_m: 10.0, height_m: 8.0 },
  go2_pose: { x_m: 3.5, y_m: 2.0, yaw_deg: 45.0 },
};

const obstacles = [
  { x: 2.0, y: 1.0, w: 3.0, h: 0.3 },
  { x: 6.0, y: 3.0, w: 0.4, h: 2.5 },
  { x: 1.0, y: 5.5, w: 2.8, h: 0.4 },
];

async function loadConfig() {
  const res = await fetch("../config/go2_map.json");
  if (!res.ok) {
    return defaultConfig;
  }
  return await res.json();
}

function createRobot(x, y, yawDeg) {
  const group = new THREE.Group();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(0.7, 0.28, 0.35),
    new THREE.MeshStandardMaterial({ color: 0x2d7ff9 })
  );
  body.position.y = 0.18;
  group.add(body);

  const heading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.4, 0),
    0.8,
    0xff6f00,
    0.24,
    0.12
  );
  group.add(heading);

  group.position.set(x, 0, y);
  group.rotation.y = -THREE.MathUtils.degToRad(yawDeg);
  return group;
}

function createObstacle(o) {
  const mesh = new THREE.Mesh(
    new THREE.BoxGeometry(o.w, 0.3, o.h),
    new THREE.MeshStandardMaterial({ color: 0x424242 })
  );
  mesh.position.set(o.x + o.w / 2, 0.15, o.y + o.h / 2);
  return mesh;
}

async function main() {
  const cfg = await loadConfig();
  const width = cfg.map.width_m;
  const height = cfg.map.height_m;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0xf4f7fb);

  const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
  camera.position.set(width * 0.75, Math.max(width, height), height * 0.75);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(width / 2, 0, height / 2);
  controls.update();

  const ambient = new THREE.AmbientLight(0xffffff, 0.7);
  scene.add(ambient);

  const dir = new THREE.DirectionalLight(0xffffff, 1.0);
  dir.position.set(4, 10, 3);
  scene.add(dir);

  const floor = new THREE.Mesh(
    new THREE.PlaneGeometry(width, height),
    new THREE.MeshStandardMaterial({ color: 0xe7ecf3, side: THREE.DoubleSide })
  );
  floor.rotation.x = -Math.PI / 2;
  floor.position.set(width / 2, 0, height / 2);
  scene.add(floor);

  const grid = new THREE.GridHelper(Math.max(width, height), Math.max(width, height), 0xa0aec0, 0xcbd5e1);
  grid.position.set(width / 2, 0.001, height / 2);
  scene.add(grid);

  for (const o of obstacles) {
    scene.add(createObstacle(o));
  }

  const robot = createRobot(cfg.go2_pose.x_m, cfg.go2_pose.y_m, cfg.go2_pose.yaw_deg);
  scene.add(robot);

  window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  renderer.setAnimationLoop(() => {
    renderer.render(scene, camera);
  });
}

main();
