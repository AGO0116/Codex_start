import * as THREE from "three";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import {
  plannerModules,
  inflateBounds,
  pointInRect,
  projectPointOntoPath,
  samplePathAtProgress,
} from "./modules/path-planners.js";
import { followerModules } from "./modules/path-followers.js";
import { planTilesRoute } from "./modules/tiles-route-planner.js";
import {
  createRuntimeMapState,
} from "./modules/map-state.js";
import { drawPlannerDebugView, showPlannerDebugPanel } from "./modules/planner-debug-view.js";

function loadStoredMapState() {
  return null;
}

function saveStoredMapState() {
  // Tiles mode is isolated from the sandbox/editor local map state on purpose.
}

const defaultConfig = {
  map: { width_m: 10.0, height_m: 8.0 },
  go2_pose: { x_m: 3.5, y_m: 2.0, yaw_deg: 45.0 },
};
const urdfBasis = new THREE.Matrix4().set(
  1, 0, 0, 0,
  0, 0, 1, 0,
  0, -1, 0, 0,
  0, 0, 0, 1
);
const urdfBasisInverse = urdfBasis.clone().invert();
const hipMeshCorrection = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 0, 0),
  Math.PI * 1.5
);
const turtlebotBaseMeshCorrection = new THREE.Quaternion().setFromAxisAngle(
  new THREE.Vector3(1, 0, 0),
  Math.PI / 2
);
const turtlebotBaseVerticalOffset = 0.014;
const turtlebotWheelCenterHeight = 0.023;

const go2Spec = {
  trunk: { length: 0.3762, width: 0.0935, height: 0.114 },
  standing: { length: 0.7, width: 0.31, height: 0.4 },
  crouching: { length: 0.76, width: 0.31, height: 0.2 },
  joints: {
    hip: { min: -1.0472, max: 1.0472 },
    frontThigh: { min: -1.5708, max: 3.4907 },
    rearThigh: { min: -0.5236, max: 4.5379 },
    calf: { min: -2.7227, max: -0.83776 },
  },
  legs: {
    FL: {
      hipOrigin: [0.1934, 0.0465, 0],
      thighOrigin: [0, 0.0955, 0],
      calfOrigin: [0, 0, -0.213],
      footOrigin: [0, 0, -0.213],
      hipVisualRpy: [0, 0, 0],
      thighAsset: "thigh",
      calfAsset: "calf",
      thighLimits: "frontThigh",
      phase: 0,
    },
    FR: {
      hipOrigin: [0.1934, -0.0465, 0],
      thighOrigin: [0, -0.0955, 0],
      calfOrigin: [0, 0, -0.213],
      footOrigin: [0, 0, -0.213],
      hipVisualRpy: [Math.PI, 0, 0],
      thighAsset: "thigh_mirror",
      calfAsset: "calf_mirror",
      thighLimits: "frontThigh",
      phase: Math.PI,
    },
    RL: {
      hipOrigin: [-0.1934, 0.0465, 0],
      thighOrigin: [0, 0.0955, 0],
      calfOrigin: [0, 0, -0.213],
      footOrigin: [0, 0, -0.213],
      hipVisualRpy: [0, Math.PI, 0],
      thighAsset: "thigh",
      calfAsset: "calf",
      thighLimits: "rearThigh",
      phase: Math.PI,
    },
    RR: {
      hipOrigin: [-0.1934, -0.0465, 0],
      thighOrigin: [0, -0.0955, 0],
      calfOrigin: [0, 0, -0.213],
      footOrigin: [0, 0, -0.213],
      hipVisualRpy: [Math.PI, Math.PI, 0],
      thighAsset: "thigh_mirror",
      calfAsset: "calf_mirror",
      thighLimits: "rearThigh",
      phase: 0,
    },
  },
};

const turtlebot3Spec = {
  name: "TurtleBot3 Burger",
  footprintRadius: 0.12,
  stepHeight: 0.04,
  canJump: false,
  wheelRadius: 0.033,
  wheelSeparation: 0.16,
};
const tilesetStorageKey = "robot-nav-viewer.tileset-url.v1";
const plateauPresets = {
  shibuya: "./data/plateau/shibuya/tileset.json",
  shinjuku: "./data/plateau/shinjuku/tileset.json",
  chiyoda: "./data/plateau/chiyoda/tileset.json",
};
const wgs84A = 6378137;
const wgs84F = 1 / 298.257223563;
const wgs84E2 = wgs84F * (2 - wgs84F);

async function loadConfig() {
  try {
    const res = await fetch("../config/go2_map.json");
    if (!res.ok) {
      return defaultConfig;
    }
    return await res.json();
  } catch {
    return defaultConfig;
  }
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

function mapToWorld(x, y, widthM, heightM) {
  return {
    x: x - widthM / 2,
    z: y - heightM / 2,
  };
}

function urdfVectorToThree(x, y, z) {
  return new THREE.Vector3(x, z, -y);
}

function urdfEulerToThree(r, p, y) {
  const urdfQuaternion = new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, "XYZ"));
  const rotationMatrix = new THREE.Matrix4().makeRotationFromQuaternion(urdfQuaternion);
  const threeMatrix = urdfBasis.clone().multiply(rotationMatrix).multiply(urdfBasisInverse);
  return new THREE.Quaternion().setFromRotationMatrix(threeMatrix);
}

function urdfAxisToThree(x, y, z) {
  return urdfVectorToThree(x, y, z).normalize();
}

function createRenderer() {
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.style.display = "block";
  renderer.domElement.style.width = "100vw";
  renderer.domElement.style.height = "100vh";
  document.body.appendChild(renderer.domElement);
  return renderer;
}

function createMapBase(scene, cfg) {
  const widthM = cfg.map.width_m;
  const heightM = cfg.map.height_m;

  const mapGroup = new THREE.Group();
  scene.add(mapGroup);

  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(widthM, 0.12, heightM),
    new THREE.MeshStandardMaterial({
      color: 0x27364a,
      roughness: 0.88,
      metalness: 0.1,
    })
  );
  floor.position.y = -0.06;
  floor.receiveShadow = true;
  mapGroup.add(floor);

  const border = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(widthM, 0.13, heightM)),
    new THREE.LineBasicMaterial({ color: 0x97a7bf, transparent: true, opacity: 0.72 })
  );
  border.position.y = -0.055;
  mapGroup.add(border);

  const grid = new THREE.GridHelper(widthM, widthM, 0x7c8da7, 0x4a5b72);
  grid.material.opacity = 0.52;
  grid.material.transparent = true;
  grid.position.y = 0.01;
  mapGroup.add(grid);

  const stripes = new THREE.Mesh(
    new THREE.PlaneGeometry(widthM * 0.95, heightM * 0.95, 1, 1),
    new THREE.MeshBasicMaterial({
      color: 0xff7a1a,
      transparent: true,
      opacity: 0.08,
      side: THREE.DoubleSide,
    })
  );
  stripes.rotation.x = -Math.PI / 2;
  stripes.position.y = 0.015;
  mapGroup.add(stripes);
  return mapGroup;
}

function createObstacles(scene, cfg, obstacleList) {
  const obstacleGroup = new THREE.Group();
  const widthM = cfg.map.width_m;
  const heightM = cfg.map.height_m;
  const bounds = [];

  const material = new THREE.MeshStandardMaterial({
    color: 0x9fd7ff,
    roughness: 0.72,
    metalness: 0.18,
  });
  const meshes = [];

  for (const obstacle of obstacleList) {
    const footprint = mapToWorld(
      obstacle.x + obstacle.w / 2,
      obstacle.y + obstacle.d / 2,
      widthM,
      heightM
    );

    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(obstacle.w, obstacle.h, obstacle.d),
      material.clone()
    );
    mesh.position.set(footprint.x, (obstacle.elevation ?? 0) + obstacle.h / 2, footprint.z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.userData.obstacleId = obstacle.id;
    obstacleGroup.add(mesh);
    meshes.push(mesh);
    bounds.push({
      minX: footprint.x - obstacle.w / 2,
      maxX: footprint.x + obstacle.w / 2,
      minZ: footprint.z - obstacle.d / 2,
      maxZ: footprint.z + obstacle.d / 2,
      topY: (obstacle.elevation ?? 0) + obstacle.h,
      bottomY: obstacle.elevation ?? 0,
      obstacleId: obstacle.id,
    });
  }

  scene.add(obstacleGroup);
  return { obstacleGroup, bounds, meshes };
}

function createScanRing() {
  const ringGroup = new THREE.Group();
  const radii = [0.45, 0.78, 1.08];

  for (let index = 0; index < radii.length; index += 1) {
    const radius = radii[index];
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(radius - 0.015, radius, 64),
      new THREE.MeshBasicMaterial({
        color: 0xff7a1a,
        transparent: true,
        opacity: 0.18 - index * 0.04,
        side: THREE.DoubleSide,
      })
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.016 + index * 0.001;
    ringGroup.add(ring);
  }

  return ringGroup;
}

function createLights(scene, cfg) {
  const hemi = new THREE.HemisphereLight(0xf2f7ff, 0x122031, 1.55);
  scene.add(hemi);

  const spot = new THREE.SpotLight(0xfff0df, 3.4, 0, Math.PI / 5, 0.3, 1);
  spot.position.set(cfg.map.width_m * 0.35, 8, cfg.map.height_m * 0.15);
  spot.target.position.set(0, 0, 0);
  spot.castShadow = true;
  spot.shadow.mapSize.set(1024, 1024);
  scene.add(spot);
  scene.add(spot.target);

  const rim = new THREE.DirectionalLight(0xffa45c, 1.15);
  rim.position.set(-4, 3.5, -2);
  scene.add(rim);
}

function loadColladaScene(loader, url) {
  return new Promise((resolve, reject) => {
    loader.load(
      url,
      (collada) => resolve(collada.scene),
      undefined,
      (error) => reject(error)
    );
  });
}

function configureMeshTree(object) {
  object.traverse((node) => {
    if (node.isMesh) {
      node.castShadow = true;
      node.receiveShadow = true;
      if (Array.isArray(node.material)) {
        for (const material of node.material) {
          material.side = THREE.DoubleSide;
        }
      } else if (node.material) {
        node.material.side = THREE.DoubleSide;
      }
    }
  });
}

async function loadGo2Assets() {
  const loader = new ColladaLoader();
  const paths = {
    base: "./assets/go2/base.dae",
    hip: "./assets/go2/hip.dae",
    thigh: "./assets/go2/thigh.dae",
    thigh_mirror: "./assets/go2/thigh_mirror.dae",
    calf: "./assets/go2/calf.dae",
    calf_mirror: "./assets/go2/calf_mirror.dae",
    foot: "./assets/go2/foot.dae",
  };

  const entries = await Promise.all(
    Object.entries(paths).map(async ([key, path]) => [key, await loadColladaScene(loader, path)])
  );

  const assets = Object.fromEntries(entries);
  for (const asset of Object.values(assets)) {
    configureMeshTree(asset);
  }
  return assets;
}

function loadStlGeometry(loader, url) {
  return new Promise((resolve, reject) => {
    loader.load(url, resolve, undefined, reject);
  });
}

function makeEcefToLocalFrame(center) {
  if (center.lengthSq() < 1e-6) {
    return new THREE.Matrix4();
  }

  const up = center.clone().normalize();
  const worldNorth = new THREE.Vector3(0, 0, 1);
  const worldAlt = new THREE.Vector3(0, 1, 0);
  const east = new THREE.Vector3().crossVectors(worldNorth, up);

  if (east.lengthSq() < 1e-10) {
    east.crossVectors(worldAlt, up);
  }
  east.normalize();

  const north = new THREE.Vector3().crossVectors(up, east).normalize();

  return new THREE.Matrix4().set(
    east.x, east.y, east.z, 0,
    up.x, up.y, up.z, 0,
    north.x, north.y, north.z, 0,
    0, 0, 0, 1
  );
}

function makeEcefAxes(center) {
  if (center.lengthSq() < 1e-6) {
    return {
      east: new THREE.Vector3(1, 0, 0),
      up: new THREE.Vector3(0, 1, 0),
      north: new THREE.Vector3(0, 0, 1),
    };
  }

  const up = center.clone().normalize();
  const worldNorth = new THREE.Vector3(0, 0, 1);
  const worldAlt = new THREE.Vector3(0, 1, 0);
  const east = new THREE.Vector3().crossVectors(worldNorth, up);
  if (east.lengthSq() < 1e-10) {
    east.crossVectors(worldAlt, up);
  }
  east.normalize();
  const north = new THREE.Vector3().crossVectors(up, east).normalize();
  return { east, up, north };
}

function geodeticToEcef(longitude, latitude, height = 0) {
  const sinLat = Math.sin(latitude);
  const cosLat = Math.cos(latitude);
  const sinLon = Math.sin(longitude);
  const cosLon = Math.cos(longitude);
  const normal = wgs84A / Math.sqrt(1 - wgs84E2 * sinLat * sinLat);

  return new THREE.Vector3(
    (normal + height) * cosLat * cosLon,
    (normal + height) * cosLat * sinLon,
    (normal * (1 - wgs84E2) + height) * sinLat
  );
}

async function loadTilesetRegionFrame(url) {
  try {
    const response = await fetch(url);
    if (!response.ok) {
      return null;
    }
    const tileset = await response.json();
    const region = tileset?.root?.boundingVolume?.region;
    if (!Array.isArray(region) || region.length < 6) {
      return null;
    }

    const [west, south, east, north, minHeight, maxHeight] = region;
    const centerLongitude = (west + east) * 0.5;
    const centerLatitude = (south + north) * 0.5;
    const centerHeight = (minHeight + maxHeight) * 0.5;
    const center = geodeticToEcef(centerLongitude, centerLatitude, centerHeight);
    const axes = makeEcefAxes(center);
    let radius = 1;

    for (const longitude of [west, east]) {
      for (const latitude of [south, north]) {
        for (const height of [minHeight, maxHeight]) {
          const centered = geodeticToEcef(longitude, latitude, height).sub(center);
          const local = new THREE.Vector3(
            centered.dot(axes.east),
            centered.dot(axes.up),
            centered.dot(axes.north)
          );
          radius = Math.max(radius, local.length());
        }
      }
    }

    return { center, radius, ...axes };
  } catch {
    return null;
  }
}

function loadStoredTilesetUrl() {
  try {
    return window.localStorage.getItem(tilesetStorageKey) ?? "./data/plateau/shibuya/tileset.json";
  } catch {
    return "./data/plateau/shibuya/tileset.json";
  }
}

function saveStoredTilesetUrl(url) {
  try {
    window.localStorage.setItem(tilesetStorageKey, url);
  } catch {
    // Ignore localStorage errors.
  }
}

async function loadTurtlebot3Assets() {
  const loader = new STLLoader();
  const paths = {
    base: "./assets/turtlebot3/burger_base.stl",
    leftWheel: "./assets/turtlebot3/left_tire.stl",
    rightWheel: "./assets/turtlebot3/right_tire.stl",
    lidar: "./assets/turtlebot3/lds.stl",
  };

  const entries = await Promise.all(
    Object.entries(paths).map(async ([key, path]) => [key, await loadStlGeometry(loader, path)])
  );

  return Object.fromEntries(entries);
}

function cloneAsset(asset) {
  const clone = asset.clone(true);
  configureMeshTree(clone);
  return clone;
}

function addPrimitiveSensors(robotFrame) {
  const lidar = new THREE.Mesh(
    new THREE.CylinderGeometry(0.048, 0.048, 0.045, 24),
    new THREE.MeshStandardMaterial({
      color: 0xc9d1dd,
      roughness: 0.22,
      metalness: 0.78,
    })
  );
  lidar.position.copy(urdfVectorToThree(0.285, 0, 0.01));
  lidar.rotation.z = Math.PI / 2;
  lidar.castShadow = true;
  robotFrame.add(lidar);

  const cameraPod = new THREE.Mesh(
    new THREE.SphereGeometry(0.047, 24, 24),
    new THREE.MeshStandardMaterial({
      color: 0x0c1018,
      roughness: 0.55,
      metalness: 0.18,
    })
  );
  cameraPod.position.copy(urdfVectorToThree(0.293, 0, -0.06));
  cameraPod.castShadow = true;
  robotFrame.add(cameraPod);
}

function createFallbackBody() {
  const group = new THREE.Group();

  const trunk = new THREE.Mesh(
    new THREE.BoxGeometry(go2Spec.trunk.length, go2Spec.trunk.height, 0.22),
    new THREE.MeshStandardMaterial({
      color: 0x171d28,
      roughness: 0.4,
      metalness: 0.42,
    })
  );
  trunk.castShadow = true;
  trunk.receiveShadow = true;
  group.add(trunk);

  const accent = new THREE.Mesh(
    new THREE.BoxGeometry(0.18, 0.08, 0.14),
    new THREE.MeshStandardMaterial({
      color: 0xff7a1a,
      roughness: 0.34,
      metalness: 0.2,
      emissive: 0x8a3d00,
      emissiveIntensity: 0.2,
    })
  );
  accent.position.copy(urdfVectorToThree(0.2, 0, 0.015));
  accent.castShadow = true;
  group.add(accent);

  return group;
}

function createTurtlebotMesh(geometry, material, scale = 0.001) {
  const mesh = new THREE.Mesh(geometry, material);
  mesh.scale.setScalar(scale);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

function applyUrdfQuaternion(object, r, p, y) {
  object.quaternion.copy(urdfEulerToThree(r, p, y));
}

function createTurtlebot3Model(assets) {
  const root = new THREE.Group();
  const baseFrame = new THREE.Group();
  baseFrame.position.copy(urdfVectorToThree(0, 0, 0.01));
  root.add(baseFrame);
  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: 0x4c525b,
    roughness: 0.58,
    metalness: 0.18,
  });
  const wheelMaterial = new THREE.MeshStandardMaterial({
    color: 0x1f2329,
    roughness: 0.88,
    metalness: 0.08,
  });
  const lidarMaterial = new THREE.MeshStandardMaterial({
    color: 0x2f343d,
    roughness: 0.42,
    metalness: 0.25,
  });

  if (assets?.base) {
    const base = createTurtlebotMesh(assets.base, bodyMaterial);
    base.position.copy(urdfVectorToThree(-0.032, 0, 0));
    base.position.y += turtlebotBaseVerticalOffset;
    applyUrdfQuaternion(base, 0, 0, 0);
    base.quaternion.multiply(turtlebotBaseMeshCorrection);
    baseFrame.add(base);
  } else {
    const fallback = new THREE.Mesh(
      new THREE.CylinderGeometry(0.09, 0.09, 0.12, 28),
      bodyMaterial
    );
    fallback.castShadow = true;
    fallback.receiveShadow = true;
    fallback.position.y = 0.06;
    baseFrame.add(fallback);
  }

  const wheelLeftMount = new THREE.Group();
  wheelLeftMount.position.copy(urdfVectorToThree(0, 0.08, 0.023));
  applyUrdfQuaternion(wheelLeftMount, -1.57, 0, 0);
  baseFrame.add(wheelLeftMount);

  const wheelRightMount = new THREE.Group();
  wheelRightMount.position.copy(urdfVectorToThree(0, -0.08, 0.023));
  applyUrdfQuaternion(wheelRightMount, -1.57, 0, 0);
  baseFrame.add(wheelRightMount);

  const wheelLeftSpin = new THREE.Group();
  const wheelRightSpin = new THREE.Group();
  wheelLeftMount.add(wheelLeftSpin);
  wheelRightMount.add(wheelRightSpin);

  if (assets?.leftWheel) {
    const leftWheel = createTurtlebotMesh(assets.leftWheel, wheelMaterial);
    wheelLeftSpin.add(leftWheel);
  }

  if (assets?.rightWheel) {
    const rightWheel = createTurtlebotMesh(assets.rightWheel, wheelMaterial);
    wheelRightSpin.add(rightWheel);
  }

  const caster = new THREE.Mesh(
    new THREE.BoxGeometry(0.03, 0.02, 0.009),
    new THREE.MeshStandardMaterial({
      color: 0x3c4048,
      roughness: 0.9,
      metalness: 0.08,
    })
  );
  caster.position.copy(urdfVectorToThree(-0.081, 0, -0.004));
  caster.castShadow = true;
  caster.receiveShadow = true;
  baseFrame.add(caster);

  if (assets?.lidar) {
    const lidar = createTurtlebotMesh(assets.lidar, lidarMaterial);
    lidar.position.copy(urdfVectorToThree(-0.032, 0, 0.172));
    applyUrdfQuaternion(lidar, 0, 0, 0);
    lidar.quaternion.multiply(turtlebotBaseMeshCorrection);
    baseFrame.add(lidar);
  }

  const heading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.06, 0),
    0.22,
    0x7ed0ff,
    0.08,
    0.04
  );
  root.add(heading);

  return {
    root,
    wheelLeftSpin,
    wheelRightSpin,
  };
}

function createGo2Model(assets) {
  const root = new THREE.Group();
  const robotFrame = new THREE.Group();
  root.add(robotFrame);

  if (assets?.base) {
    robotFrame.add(cloneAsset(assets.base));
  } else {
    robotFrame.add(createFallbackBody());
  }

  addPrimitiveSensors(robotFrame);

  const legMap = {};
  const jointAxes = {
    hip: urdfAxisToThree(1, 0, 0),
    pitch: urdfAxisToThree(0, 1, 0),
  };

  for (const [name, spec] of Object.entries(go2Spec.legs)) {
    const hipPivot = new THREE.Group();
    hipPivot.position.copy(urdfVectorToThree(...spec.hipOrigin));
    robotFrame.add(hipPivot);

    if (assets?.hip) {
      const hipMesh = cloneAsset(assets.hip);
      hipMesh.quaternion.copy(urdfEulerToThree(...spec.hipVisualRpy));
      hipMesh.quaternion.multiply(hipMeshCorrection);
      hipPivot.add(hipMesh);
    }

    const thighPivot = new THREE.Group();
    thighPivot.position.copy(urdfVectorToThree(...spec.thighOrigin));
    hipPivot.add(thighPivot);

    const thighAsset = assets?.[spec.thighAsset];
    if (thighAsset) {
      thighPivot.add(cloneAsset(thighAsset));
    }

    const calfPivot = new THREE.Group();
    calfPivot.position.copy(urdfVectorToThree(...spec.calfOrigin));
    thighPivot.add(calfPivot);

    const calfAsset = assets?.[spec.calfAsset];
    if (calfAsset) {
      calfPivot.add(cloneAsset(calfAsset));
    }

    const footAnchor = new THREE.Group();
    footAnchor.position.copy(urdfVectorToThree(...spec.footOrigin));
    calfPivot.add(footAnchor);

    if (assets?.foot) {
      footAnchor.add(cloneAsset(assets.foot));
    }

    legMap[name] = {
      phase: spec.phase,
      isFront: name.startsWith("F"),
      isLeft: name.endsWith("L"),
      thighLimits: go2Spec.joints[spec.thighLimits],
      hipPivot,
      thighPivot,
      calfPivot,
      footAnchor,
      hipAxis: jointAxes.hip,
      thighAxis: jointAxes.pitch,
      calfAxis: jointAxes.pitch,
      pose: { hip: 0, thigh: 0, calf: 0 },
    };
  }

  const heading = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.14, 0),
    0.54,
    0xffb176,
    0.12,
    0.07
  );
  root.add(heading);

  return { root, legs: legMap };
}

function applyJointPose(leg, pose) {
  leg.pose = pose;
  leg.hipPivot.quaternion.setFromAxisAngle(leg.hipAxis, pose.hip);
  leg.thighPivot.quaternion.setFromAxisAngle(leg.thighAxis, pose.thigh);
  leg.calfPivot.quaternion.setFromAxisAngle(leg.calfAxis, pose.calf);
}

function updateGo2Pose(go2Rig, motionState, elapsed, moveState) {
  const locomotionAmount = Math.max(
    moveState.forwardAmount,
    moveState.lateralAmount,
    moveState.turnAmount
  );
  const stridePhase = elapsed * (locomotionAmount > 0.01 ? moveState.cadence : 0.8);
  const crouchBlend = moveState.sneak ? 1 : 0;
  const jumpBlend = motionState.jumpY > 0 ? clamp(motionState.jumpY / 0.7, 0, 1) : 0;

  const baseThigh = THREE.MathUtils.lerp(0.92, 1.3, crouchBlend);
  const baseCalf = THREE.MathUtils.lerp(-1.82, -2.3, crouchBlend);
  const baseHip = THREE.MathUtils.lerp(0, 0.08, crouchBlend);
  const stepAmplitude = locomotionAmount > 0.01 ? THREE.MathUtils.lerp(0.12, 0.42, locomotionAmount) : 0;
  const calfAmplitude = stepAmplitude * 0.85;
  const lateralLean = moveState.lateral * 0.14;
  const jumpTuck = jumpBlend * 0.45;

  for (const leg of Object.values(go2Rig.legs)) {
    const phase = stridePhase + leg.phase;
    const swing = Math.sin(phase);
    const lift = Math.max(0, Math.sin(phase + Math.PI / 4));
    const sideSign = leg.isLeft ? 1 : -1;
    const frontSign = leg.isFront ? 1 : -1;

    // Keep the forward trot as the base rhythm, then layer sidestep spread
    // and turning twist on top so the gait stays physically coherent.
    const forwardSwing = swing * moveState.forward * 0.95;

    // Sidestepping should open the stance laterally while keeping diagonal timing.
    const lateralSwing = swing * moveState.lateral * frontSign * 0.2;
    const lateralHipStride = lift * moveState.lateral * 0.26;
    const lateralLift = lift * moveState.lateralAmount * 0.62;

    // Turning in place reuses the trot cadence, but twists left/right legs oppositely.
    const turnSwing = swing * moveState.turn * sideSign * 0.42;
    const turnHipStride = swing * moveState.turn * sideSign * 0.34;
    const turnLift = lift * moveState.turnAmount * 0.72;

    const compositeSwing = forwardSwing + lateralSwing + turnSwing;
    const compositeLift =
      lift * moveState.forwardAmount * 0.9 +
      lateralLift +
      turnLift;

    const turnHipBias = moveState.turn * sideSign * 0.08;
    const lateralHipBias = moveState.lateral * 0.16;

    const hip = clamp(
      baseHip * sideSign +
      lateralLean * sideSign +
      lateralHipStride +
      turnHipStride +
      lateralHipBias +
      turnHipBias,
      go2Spec.joints.hip.min,
      go2Spec.joints.hip.max
    );
    const thigh = clamp(
      baseThigh + compositeSwing * stepAmplitude + jumpTuck,
      leg.thighLimits.min,
      leg.thighLimits.max
    );
    const calf = clamp(
      baseCalf - compositeLift * calfAmplitude - jumpTuck * 1.8,
      go2Spec.joints.calf.min,
      go2Spec.joints.calf.max
    );

    applyJointPose(leg, { hip, thigh, calf });
  }
}

function calibrateGroundOffset(go2Rig) {
  go2Rig.root.updateMatrixWorld(true);
  const bounds = new THREE.Box3();
  let minY = Infinity;

  for (const leg of Object.values(go2Rig.legs)) {
    bounds.setFromObject(leg.footAnchor);
    minY = Math.min(minY, bounds.min.y);
  }

  if (!Number.isFinite(minY)) {
    return go2Spec.standing.height * 0.5;
  }

  return -minY + 0.002;
}

function calibrateGenericGroundOffset(root) {
  root.updateMatrixWorld(true);
  const bounds = new THREE.Box3().setFromObject(root);
  if (!Number.isFinite(bounds.min.y)) {
    return 0;
  }
  return -bounds.min.y + 0.002;
}

function placeGo2(root, cfg) {
  const world = mapToWorld(cfg.go2_pose.x_m, cfg.go2_pose.y_m, cfg.map.width_m, cfg.map.height_m);
  root.position.set(world.x, 0, world.z);
  root.rotation.y = (-cfg.go2_pose.yaw_deg * Math.PI) / 180;
}

function createScene(cfg, obstacleList) {
  const scene = new THREE.Scene();
  const defaultFog = new THREE.Fog(0x05070b, 18, 54);
  scene.fog = defaultFog;

  const mapGroup = createMapBase(scene, cfg);
  const { obstacleGroup, bounds: obstacleBounds, meshes: obstacleMeshes } = createObstacles(
    scene,
    cfg,
    obstacleList
  );
  createLights(scene, cfg);

  const scanRing = createScanRing();
  scene.add(scanRing);

  return {
    scene,
    scanRing,
    obstacleBounds,
    mapGroup,
    obstacleGroup,
    obstacleMeshes,
    tilesRenderer: null,
    tilesGroupOffset: new THREE.Group(),
    defaultFog,
  };
}

function updateHud(cfg, platform = "Unitree Go2") {
  const platformNode = document.querySelector('[data-role="platform"]');
  const renderNode = document.querySelector('[data-role="render"]');
  const mapNode = document.querySelector('[data-role="map"]');
  const poseNode = document.querySelector('[data-role="pose"]');

  if (platformNode) {
    platformNode.textContent = platform;
  }

  if (renderNode) {
    renderNode.textContent = platform.includes("TurtleBot3")
      ? "Wheel Robot + Navigation View"
      : "Legged Robot + Navigation View";
  }

  if (mapNode) {
    mapNode.textContent = `${cfg.map.width_m}m x ${cfg.map.height_m}m`;
  }

  if (poseNode) {
    poseNode.textContent = `x ${cfg.go2_pose.x_m} / y ${cfg.go2_pose.y_m} / yaw ${cfg.go2_pose.yaw_deg}deg`;
  }
}

function updateLoadingState(text) {
  const renderNode = document.querySelector('[data-role="render"]');
  if (renderNode) {
    renderNode.textContent = text;
  }
}

function updatePlannerStatus(text) {
  const node = document.querySelector('[data-role="planner-status"]');
  if (node) {
    node.textContent = text;
  }
}

function setButtonActive(id, isActive) {
  const button = document.getElementById(id);
  if (button) {
    button.classList.toggle("is-active", isActive);
  }
}

function circleIntersectsAabb(x, z, radius, box) {
  const nearestX = clamp(x, box.minX, box.maxX);
  const nearestZ = clamp(z, box.minZ, box.maxZ);
  const dx = x - nearestX;
  const dz = z - nearestZ;
  return dx * dx + dz * dz < radius * radius;
}

function resolveCollisions(nextX, nextZ, currentX, currentZ, radius, obstacleBounds) {
  let resolvedX = nextX;
  let resolvedZ = currentZ;

  for (const box of obstacleBounds) {
    if (circleIntersectsAabb(resolvedX, resolvedZ, radius, box)) {
      resolvedX = currentX;
      break;
    }
  }

  resolvedZ = nextZ;
  for (const box of obstacleBounds) {
    if (circleIntersectsAabb(resolvedX, resolvedZ, radius, box)) {
      resolvedZ = currentZ;
      break;
    }
  }

  return { x: resolvedX, z: resolvedZ };
}

function evaluateSupportSurface(x, z, radius, currentSupportY, stepHeight, obstacleBounds) {
  let supportY = 0;

  for (const box of obstacleBounds) {
    if (!circleIntersectsAabb(x, z, radius, box)) {
      continue;
    }

    if ((box.bottomY ?? 0) > currentSupportY + stepHeight + 1e-4) {
      continue;
    }

    if (box.topY > currentSupportY + stepHeight + 1e-4) {
      return { blocked: true, supportY: currentSupportY };
    }

    supportY = Math.max(supportY, box.topY);
  }

  return { blocked: false, supportY };
}

function resolveMotionWithSteps(nextX, nextZ, currentX, currentZ, currentSupportY, radius, stepHeight, obstacleBounds) {
  let resolvedX = currentX;
  let resolvedZ = currentZ;
  let resolvedSupportY = currentSupportY;

  const xTest = evaluateSupportSurface(nextX, currentZ, radius, currentSupportY, stepHeight, obstacleBounds);
  if (!xTest.blocked) {
    resolvedX = nextX;
    resolvedSupportY = xTest.supportY;
  }

  const zTest = evaluateSupportSurface(resolvedX, nextZ, radius, resolvedSupportY, stepHeight, obstacleBounds);
  if (!zTest.blocked) {
    resolvedZ = nextZ;
    resolvedSupportY = zTest.supportY;
  }

  const finalTest = evaluateSupportSurface(
    resolvedX,
    resolvedZ,
    radius,
    Math.max(currentSupportY, resolvedSupportY),
    stepHeight,
    obstacleBounds
  );

  return {
    x: resolvedX,
    z: resolvedZ,
    supportY: finalTest.blocked ? currentSupportY : finalTest.supportY,
  };
}

function createMarker(color, radius, height) {
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, height, 24),
    new THREE.MeshStandardMaterial({
      color,
      emissive: color,
      emissiveIntensity: 0.18,
      roughness: 0.38,
      metalness: 0.15,
    })
  );
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.visible = false;
  return mesh;
}

function pointToVector(point, y) {
  return new THREE.Vector3(point.x, y, point.z);
}

function buildPathRenderables(path, robotType) {
  const group = new THREE.Group();

  const polyPoints = path.map((point) => pointToVector(point, 0.06));
  if (robotType === "go2") {
    const polyline = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(polyPoints),
      new THREE.LineBasicMaterial({
        color: 0x1d8f4d,
        transparent: true,
        opacity: 0.95,
      })
    );
    group.add(polyline);
  }

  let curvePoints = polyPoints;
  if (path.length >= 3) {
    const curve = new THREE.CatmullRomCurve3(polyPoints, false, "centripetal", 0.15);
    curvePoints = curve.getPoints(Math.max(32, path.length * 18));
  }

  if (robotType === "turtlebot3") {
    const curveLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(curvePoints.map((point) => point.clone().setY(0.08))),
      new THREE.LineBasicMaterial({
        color: 0xffb176,
        transparent: true,
        opacity: 0.95,
      })
    );
    group.add(curveLine);
  }

  return group;
}

export async function main(appModeOverride) {
  const cfg = await loadConfig();
  const appMode = appModeOverride ?? document.body.dataset.appMode ?? "viewer";
  const storedMapState = loadStoredMapState();
  updateLoadingState("Loading official Go2 assets...");

  let go2Assets = null;
  let turtlebot3Assets = null;
  try {
    [go2Assets, turtlebot3Assets] = await Promise.all([loadGo2Assets(), loadTurtlebot3Assets()]);
  } catch {
    updateLoadingState("Some meshes failed to load, using fallback parts where needed");
  }

  const renderer = createRenderer();
  const camera = new THREE.PerspectiveCamera(
    46,
    window.innerWidth / window.innerHeight,
    0.1,
    180
  );

  const mapState = createRuntimeMapState(storedMapState);
  saveStoredMapState(cfg, mapState);
  const sceneRefs = createScene(cfg, mapState.obstacles);
  const modeState = {
    mapMode: "sandbox",
    tilesetUrl: loadStoredTilesetUrl(),
    tilesetLoaded: false,
    tilesetPanelOpen: appMode === "tiles",
    tileCollisionMeshes: new Set(),
    tilesSpawned: false,
    tilesSpawnArmed: false,
    positionPanelOpen: false,
    tilesFrame: null,
    tilesRobotAnchorLocal: null,
    tilesRobotAnchorWorld: null,
  };
  const { scene, scanRing } = sceneRefs;
  const go2Rig = createGo2Model(go2Assets);
  const turtlebot3Rig = createTurtlebot3Model(turtlebot3Assets);
  placeGo2(go2Rig.root, cfg);
  placeGo2(turtlebot3Rig.root, cfg);
  scene.add(go2Rig.root);
  scene.add(turtlebot3Rig.root);
  updateHud(cfg);
  let tilesSpawnRequest = null;
  let tilesRefreshFrames = 0;

  const inputState = {
    forward: false,
    backward: false,
    left: false,
    right: false,
    turnLeft: false,
    turnRight: false,
    sprint: false,
    sneak: false,
  };

  const motionState = {
    x: go2Rig.root.position.x,
    z: go2Rig.root.position.z,
    groundOffset: 0,
    supportY: 0,
    jumpY: 0,
    velocityY: 0,
    isGrounded: true,
    yaw: go2Rig.root.rotation.y,
  };
  const sandboxMotionState = { ...motionState };
  const tilesMotionState = {
    x: 0,
    z: 0,
    groundOffset: 0,
    supportY: 0,
    jumpY: 0,
    velocityY: 0,
    isGrounded: true,
    yaw: go2Rig.root.rotation.y,
  };
  const walkSpeed = 2.15;
  const sprintSpeed = 3.45;
  const sneakSpeed = 1.35;
  const turnSpeed = 2.2;
  const jumpSpeed = 3.8;
  const gravity = 9.2;
  const robotCollisionRadius = 0.24;
  const stepHeight = 0.18;
  const jumpStepBonus = 0.18;
  const autoNavSpeed = 1.2;
  const tilesStepHeight = 0.68;
  const robots = {
    go2: {
      type: "go2",
      label: "Unitree Go2",
      root: go2Rig.root,
      rig: go2Rig,
      collisionRadius: robotCollisionRadius,
      stepHeight,
      canJump: true,
      groundOffset: 0,
    },
    turtlebot3: {
      type: "turtlebot3",
      label: "TurtleBot3 Burger",
      root: turtlebot3Rig.root,
      rig: turtlebot3Rig,
      collisionRadius: turtlebot3Spec.footprintRadius,
      stepHeight: turtlebot3Spec.stepHeight,
      canJump: false,
      groundOffset: 0,
    },
  };
  let activeRobot = robots.go2;

  let obstacleBounds = sceneRefs.obstacleBounds;
  let halfWidth = cfg.map.width_m / 2 - 0.4;
  let halfHeight = cfg.map.height_m / 2 - 0.4;
  const moveDirection = new THREE.Vector3();
  const chaseOffset = new THREE.Vector3();
  const chaseLookAt = new THREE.Vector3();
  const chasePosition = new THREE.Vector3();
  const upAxis = new THREE.Vector3(0, 1, 0);
  const cameraForward = new THREE.Vector3();
  const cameraRight = new THREE.Vector3();
  const cameraScreenUp = new THREE.Vector3();
  const robotForwardWorld = new THREE.Vector3();
  const robotRightWorld = new THREE.Vector3();
  const robotBasisMatrix = new THREE.Matrix4();
  const raycaster = new THREE.Raycaster();
  const tilesFloorRaycaster = new THREE.Raycaster();
  const tilesWallRaycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const planeHit = new THREE.Vector3();
  const tilesRayOrigin = new THREE.Vector3();
  const tilesRayDirection = new THREE.Vector3();
  const orbitState = {
    azimuth: Math.PI,
    polar: 0.92,
    distance: 3.5,
    target: new THREE.Vector3(0, 0.3, 0),
    dragging: false,
    dragMode: "orbit",
    pointerId: null,
    lastX: 0,
    lastY: 0,
    panPlaneY: 0,
    panAnchor: new THREE.Vector3(),
    panRight: new THREE.Vector3(),
    panUp: new THREE.Vector3(),
  };
  const cameraState = {
    mode: appMode === "editor" ? "free" : "follow",
  };
  const sandboxViewState = {
    cameraMode: appMode === "editor" ? "free" : "follow",
    azimuth: Math.PI,
    polar: 0.92,
    distance: 3.5,
    target: new THREE.Vector3(0, 0.3, 0),
  };
  const tilesViewState = {
    cameraMode: "free",
    azimuth: Math.PI * 0.72,
    polar: 1.02,
    distance: 80,
    target: new THREE.Vector3(0, 0, 0),
  };
  const guideState = {
    visible: true,
  };
  const autoCommandState = {
    forward: 0,
    lateral: 0,
    turn: 0,
    speed: 0,
  };
  const plannerState = {
    plannerKey: "visibilityGraph",
    mode: null,
    start: null,
    goal: null,
    path: null,
    polylineMetrics: null,
    smoothPath: null,
    smoothPathMetrics: null,
    debugData: null,
    autoActive: false,
    followerKey: null,
    followerState: null,
  };
  const editorState = {
    isOpen: false,
    mode: "select",
    selectedObstacleId: null,
    armedObstacleId: null,
    armedPointerId: null,
    armedOffsetX: 0,
    armedOffsetY: 0,
    armedStartX: 0,
    armedStartY: 0,
    armedPlaneY: 0,
    draggingObstacleId: null,
    dragPointerId: null,
    dragOffsetX: 0,
    dragOffsetY: 0,
    resizingHandleAxis: null,
    resizePointerId: null,
    resizeObstacleId: null,
    resizePlaneY: 0,
    resizeStartClientY: 0,
    resizeBaseHeight: 0,
  };
  const startMarker = createMarker(0x4de18b, 0.1, 0.12);
  const goalMarker = createMarker(0xff7a7a, 0.1, 0.12);
  const routeGroup = new THREE.Group();
  const selectionOverlayGroup = new THREE.Group();
  const selectionHandleMeshes = [];
  scene.add(startMarker);
  scene.add(goalMarker);
  scene.add(routeGroup);
  scene.add(selectionOverlayGroup);

  function copyMotionState(target, source) {
    target.x = source.x;
    target.z = source.z;
    target.groundOffset = source.groundOffset;
    target.supportY = source.supportY;
    target.jumpY = source.jumpY;
    target.velocityY = source.velocityY;
    target.isGrounded = source.isGrounded;
    target.yaw = source.yaw;
  }

  function saveCurrentViewState(mode) {
    const store = mode === "tiles3d" ? tilesViewState : sandboxViewState;
    store.cameraMode = cameraState.mode;
    store.azimuth = orbitState.azimuth;
    store.polar = orbitState.polar;
    store.distance = orbitState.distance;
    store.target.copy(orbitState.target);
  }

  function loadViewState(mode) {
    const store = mode === "tiles3d" ? tilesViewState : sandboxViewState;
    cameraState.mode = store.cameraMode;
    orbitState.azimuth = store.azimuth;
    orbitState.polar = store.polar;
    orbitState.distance = store.distance;
    orbitState.target.copy(store.target);
    clampOrbitTarget();
    syncCameraModeButton();
  }

  function saveCurrentModeMotionState() {
    copyMotionState(modeState.mapMode === "tiles3d" ? tilesMotionState : sandboxMotionState, motionState);
  }

  function loadModeMotionState(mode) {
    copyMotionState(motionState, mode === "tiles3d" ? tilesMotionState : sandboxMotionState);
    motionState.groundOffset = activeRobot.groundOffset;
    syncActiveRobotTransform();
  }

  function getTilesUpAxis() {
    return modeState.tilesFrame?.up ?? upAxis;
  }

  function getCurrentUpAxis() {
    return modeState.mapMode === "tiles3d" ? getTilesUpAxis() : upAxis;
  }

  function focusOrbitTargetOnRobot(offset = 0.4) {
    orbitState.target.copy(activeRobot.root.position).addScaledVector(getCurrentUpAxis(), offset);
    clampOrbitTarget();
    saveCurrentViewState(modeState.mapMode);
  }

  function updateOrbitReticle() {
    const reticle = document.getElementById("orbit-reticle");
    if (!reticle) {
      return;
    }
    // The camera always looks at orbitState.target in free/editor view, so a fixed
    // center reticle is more stable than re-projecting the target every frame.
    const visible = cameraState.mode === "free" || editorState.isOpen;
    reticle.classList.toggle("is-visible", visible);
  }

  function tilesLocalToWorld(x, y, z, target = new THREE.Vector3()) {
    if (!modeState.tilesFrame) {
      return target.set(x, y, z);
    }

    target.copy(modeState.tilesFrame.origin ?? new THREE.Vector3());
    target.addScaledVector(modeState.tilesFrame.east, x);
    target.addScaledVector(modeState.tilesFrame.up, y);
    target.addScaledVector(modeState.tilesFrame.north, z);
    return target;
  }

  function tilesAnchoredLocalToWorld(x, y, z, target = new THREE.Vector3()) {
    if (!modeState.tilesFrame || !modeState.tilesRobotAnchorLocal || !modeState.tilesRobotAnchorWorld) {
      return tilesLocalToWorld(x, y, z, target);
    }

    const anchorLocal = modeState.tilesRobotAnchorLocal;
    target.copy(modeState.tilesRobotAnchorWorld);
    target.addScaledVector(modeState.tilesFrame.east, x - anchorLocal.x);
    target.addScaledVector(modeState.tilesFrame.up, y - anchorLocal.y);
    target.addScaledVector(modeState.tilesFrame.north, z - anchorLocal.z);
    return target;
  }

  function worldToTilesLocal(point, target = new THREE.Vector3()) {
    if (!modeState.tilesFrame) {
      return target.copy(point);
    }

    const centered = point.clone().sub(modeState.tilesFrame.origin ?? new THREE.Vector3());
    target.set(
      centered.dot(modeState.tilesFrame.east),
      centered.dot(modeState.tilesFrame.up),
      centered.dot(modeState.tilesFrame.north)
    );
    return target;
  }

  function syncActiveRobotTransform() {
    for (const robot of Object.values(robots)) {
      if (modeState.mapMode === "tiles3d" && !modeState.tilesSpawned) {
        robot.root.visible = false;
        continue;
      }
      robot.root.visible = robot === activeRobot;
      if (modeState.mapMode === "tiles3d" && modeState.tilesFrame) {
        tilesAnchoredLocalToWorld(
          motionState.x,
          motionState.groundOffset + motionState.supportY + motionState.jumpY,
          motionState.z,
          robot.root.position
        );
        // Build the robot orientation from the local PLATEAU frame so rendered
        // yaw, manual controls, and autonomous path heading share one basis.
        robotForwardWorld
          .copy(modeState.tilesFrame.east)
          .multiplyScalar(Math.cos(motionState.yaw))
          .addScaledVector(modeState.tilesFrame.north, -Math.sin(motionState.yaw))
          .normalize();
        robotRightWorld.crossVectors(robotForwardWorld, modeState.tilesFrame.up).normalize();
        robotBasisMatrix.makeBasis(robotForwardWorld, modeState.tilesFrame.up, robotRightWorld);
        robot.root.quaternion.setFromRotationMatrix(robotBasisMatrix);
      } else {
        robot.root.position.set(motionState.x, motionState.groundOffset + motionState.supportY + motionState.jumpY, motionState.z);
        robot.root.rotation.set(0, motionState.yaw, 0);
      }
    }
  }

  function switchRobot(robotKey) {
    const nextRobot = robots[robotKey];
    if (!nextRobot) {
      return;
    }

    activeRobot = nextRobot;
    motionState.jumpY = 0;
    motionState.velocityY = 0;
    motionState.isGrounded = true;
    motionState.groundOffset = activeRobot.groundOffset;
    syncActiveRobotTransform();
    if (plannerState.path) {
      clearRouteRender();
      routeGroup.add(buildPathRenderables(plannerState.path, activeRobot.type));
      if (plannerState.debugData) {
        drawPlannerDebugView(plannerState.debugData, cfg, activeRobot.label);
      }
    }
    updateHud(cfg, activeRobot.label);
    if (modeState.mapMode === "tiles3d") {
      updateLoadingState(`${activeRobot.label} + 3D Tiles View`);
    }
    updatePlannerStatus(`${activeRobot.label} selected.`);
  }

  function resetRobotsToConfigPose() {
    placeGo2(go2Rig.root, cfg);
    placeGo2(turtlebot3Rig.root, cfg);
    motionState.x = go2Rig.root.position.x;
    motionState.z = go2Rig.root.position.z;
    motionState.yaw = go2Rig.root.rotation.y;
    motionState.jumpY = 0;
    motionState.velocityY = 0;
    motionState.isGrounded = true;
    motionState.supportY = 0;
    motionState.groundOffset = activeRobot.groundOffset;
    syncActiveRobotTransform();
    const startPoint = worldToMapPoint(activeRobot.root.position);
    plannerState.start = startPoint;
    setMarkerPosition(startMarker, startPoint, motionState.supportY);
    clearPathState();
  }

  function setPlannerMode(mode) {
    plannerState.mode = mode;
    setButtonActive("set-start-btn", mode === "start");
    setButtonActive("set-goal-btn", mode === "goal");
  }

  function clearRouteRender() {
    routeGroup.clear();
  }

  function setMarkerPosition(marker, point, supportY) {
    if (modeState.mapMode === "tiles3d" && modeState.tilesFrame) {
      tilesAnchoredLocalToWorld(point.x, (point.y ?? supportY ?? 0) + 0.12, point.z, marker.position);
      marker.quaternion.setFromUnitVectors(upAxis, modeState.tilesFrame.up);
      marker.visible = true;
      return;
    }
    marker.quaternion.identity();
    marker.position.set(point.x, supportY + 0.07, point.z);
    marker.visible = true;
  }

  function buildTilesPathRenderables(path) {
    const group = new THREE.Group();
    const points = path.map((point) => {
      const worldPoint = new THREE.Vector3();
      tilesAnchoredLocalToWorld(point.x, (point.y ?? 0) + 0.14, point.z, worldPoint);
      return worldPoint;
    });
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(points),
      new THREE.LineBasicMaterial({
        color: 0x1d8f4d,
        transparent: true,
        opacity: 0.98,
      })
    );
    group.add(line);

    for (const point of path) {
      const dot = new THREE.Mesh(
        new THREE.SphereGeometry(0.13, 12, 8),
        new THREE.MeshStandardMaterial({
          color: 0x4de18b,
          emissive: 0x123d25,
          roughness: 0.55,
        })
      );
      tilesAnchoredLocalToWorld(point.x, (point.y ?? 0) + 0.2, point.z, dot.position);
      group.add(dot);
    }

    return group;
  }

  function refreshSceneGeometry() {
    if (sceneRefs.mapGroup) {
      scene.remove(sceneRefs.mapGroup);
    }
    if (sceneRefs.obstacleGroup) {
      scene.remove(sceneRefs.obstacleGroup);
    }
    sceneRefs.mapGroup = createMapBase(scene, cfg);
    const obstacleResult = createObstacles(scene, cfg, mapState.obstacles);
    sceneRefs.obstacleGroup = obstacleResult.obstacleGroup;
    obstacleBounds = obstacleResult.bounds;
    sceneRefs.obstacleBounds = obstacleBounds;
    sceneRefs.obstacleMeshes = obstacleResult.meshes;
    halfWidth = cfg.map.width_m / 2 - 0.4;
    halfHeight = cfg.map.height_m / 2 - 0.4;
    syncMapModeSceneVisibility();
    rebuildSelectionOverlay();
  }

  function clearPathState() {
    plannerState.path = null;
    plannerState.polylineMetrics = null;
    plannerState.smoothPath = null;
    plannerState.smoothPathMetrics = null;
    plannerState.debugData = null;
    plannerState.followerKey = null;
    plannerState.followerState = null;
    clearRouteRender();
    showTilesRouteDebugPanel(false);
    showPlannerDebugPanel(false);
    stopAutoNav();
  }

  function syncTilesetControls() {
    const tilesetInput = document.getElementById("tileset-url-input");
    if (tilesetInput && tilesetInput !== document.activeElement) {
      tilesetInput.value = modeState.tilesetUrl;
    }

    const presetSelect = document.getElementById("tileset-preset-select");
    if (presetSelect && presetSelect !== document.activeElement) {
      const presetEntry = Object.entries(plateauPresets).find(([, url]) => url === modeState.tilesetUrl);
      presetSelect.value = presetEntry?.[1] ?? "";
    }

    const panel = document.getElementById("tileset-panel");
    if (panel) {
      panel.classList.toggle("is-hidden", appMode === "tiles" ? false : !modeState.tilesetPanelOpen);
    }

    setButtonActive("tiles-mode-btn", modeState.tilesetPanelOpen);
    setButtonActive("tileset-sandbox-btn", modeState.mapMode === "sandbox");
    setButtonActive("tiles-spawn-btn", modeState.tilesSpawnArmed);
    setButtonActive("position-toggle-btn", modeState.positionPanelOpen);
    setButtonActive("position-float-btn", modeState.positionPanelOpen);
    setButtonActive("map-edit-btn", false);
    setButtonActive("viewer-page-btn", false);
    const spawnButton = document.getElementById("tiles-spawn-btn");
    if (spawnButton) {
      spawnButton.innerHTML = modeState.tilesSpawnArmed
        ? "<strong>Spawn Armed</strong><span>Click the target tile</span>"
        : "<strong>Spawn Robot</strong><span>Click a tile surface</span>";
    }
  }

  function syncPositionPanel() {
    const panel = document.getElementById("position-panel");
    if (!panel) {
      return;
    }
    panel.classList.toggle("is-hidden", !modeState.positionPanelOpen);
    panel.setAttribute("aria-hidden", modeState.positionPanelOpen ? "false" : "true");
    const detail = panel.querySelector('[data-role="position-detail"]');
    const mapLabel = panel.querySelector('[data-role="position-map"]');
    if (mapLabel) {
      const presetName = Object.entries(plateauPresets).find(([, url]) => url === modeState.tilesetUrl)?.[0];
      mapLabel.textContent = presetName ? presetName.toUpperCase() : "Custom";
    }
    if (!detail) {
      return;
    }
    if (modeState.mapMode === "tiles3d" && !modeState.tilesSpawned) {
      detail.textContent = "Robot is not spawned. Press Spawn Robot, then click a tile.";
      return;
    }
    detail.textContent = `local x ${motionState.x.toFixed(2)} / z ${motionState.z.toFixed(2)} / floor ${motionState.supportY.toFixed(2)} / yaw ${Math.round((-motionState.yaw * 180) / Math.PI)}deg`;
  }

  function showTilesRouteDebugPanel(visible) {
    const panel = document.getElementById("tiles-route-debug");
    if (!panel) {
      return;
    }
    panel.classList.toggle("is-visible", visible);
    panel.setAttribute("aria-hidden", visible ? "false" : "true");
  }

  function drawTilesRouteDebugView(debugData, statusText = "Search") {
    const canvas = document.getElementById("tiles-route-debug-canvas");
    if (!canvas || !debugData) {
      showTilesRouteDebugPanel(false);
      return;
    }

    const ctx = canvas.getContext("2d");
    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#07101a";
    ctx.fillRect(0, 0, width, height);

    const allPoints = [
      debugData.start,
      debugData.goal,
      ...(debugData.visitedNodes ?? []),
      ...(debugData.rawPath ?? []),
      ...(debugData.path ?? []),
    ].filter(Boolean);
    if (allPoints.length === 0) {
      return;
    }

    const pad = 16;
    const minX = Math.min(...allPoints.map((point) => point.x));
    const maxX = Math.max(...allPoints.map((point) => point.x));
    const minZ = Math.min(...allPoints.map((point) => point.z));
    const maxZ = Math.max(...allPoints.map((point) => point.z));
    const spanX = Math.max(1, maxX - minX);
    const spanZ = Math.max(1, maxZ - minZ);
    const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanZ);
    const toCanvas = (point) => ({
      x: pad + (point.x - minX) * scale,
      y: height - pad - (point.z - minZ) * scale,
    });

    ctx.strokeStyle = "rgba(126, 208, 255, 0.12)";
    ctx.lineWidth = 1;
    for (let x = Math.floor(minX); x <= Math.ceil(maxX); x += 2) {
      const p0 = toCanvas({ x, z: minZ });
      const p1 = toCanvas({ x, z: maxZ });
      ctx.beginPath();
      ctx.moveTo(p0.x, p0.y);
      ctx.lineTo(p1.x, p1.y);
      ctx.stroke();
    }
    for (let z = Math.floor(minZ); z <= Math.ceil(maxZ); z += 2) {
      const p0 = toCanvas({ x: minX, z });
      const p1 = toCanvas({ x: maxX, z });
      ctx.beginPath();
      ctx.moveTo(p0.x, p0.y);
      ctx.lineTo(p1.x, p1.y);
      ctx.stroke();
    }

    ctx.fillStyle = "rgba(126, 208, 255, 0.34)";
    for (const node of debugData.visitedNodes ?? []) {
      const p = toCanvas(node);
      ctx.fillRect(p.x - 1.3, p.y - 1.3, 2.6, 2.6);
    }

    if (debugData.rawPath?.length > 1) {
      ctx.strokeStyle = "rgba(255, 176, 118, 0.62)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      debugData.rawPath.forEach((point, index) => {
        const p = toCanvas(point);
        if (index === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
    }

    if (debugData.path?.length > 1) {
      ctx.strokeStyle = "#4de18b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      debugData.path.forEach((point, index) => {
        const p = toCanvas(point);
        if (index === 0) ctx.moveTo(p.x, p.y);
        else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
    }

    const start = toCanvas(debugData.start);
    ctx.fillStyle = "#4de18b";
    ctx.beginPath();
    ctx.arc(start.x, start.y, 5, 0, Math.PI * 2);
    ctx.fill();

    const goal = toCanvas(debugData.goal);
    ctx.fillStyle = "#ff7a7a";
    ctx.beginPath();
    ctx.arc(goal.x, goal.y, 5, 0, Math.PI * 2);
    ctx.fill();

    const modeLabel = debugData.plannerMode ? ` / ${debugData.plannerMode}` : "";
    document.querySelector('[data-role="tiles-debug-state"]').textContent = `${statusText}${modeLabel}`;
    document.querySelector('[data-role="tiles-debug-visited"]').textContent = String(debugData.visitedNodes?.length ?? 0);
    document.querySelector('[data-role="tiles-debug-raw"]').textContent = String(debugData.rawPath?.length ?? 0);
    document.querySelector('[data-role="tiles-debug-final"]').textContent = String(debugData.path?.length ?? 0);
    document.querySelector('[data-role="tiles-debug-margin"]').textContent = `${(debugData.clearanceRadius ?? 0).toFixed(2)}m`;
    showTilesRouteDebugPanel(true);
  }

  function syncMapModeSceneVisibility() {
    const sandboxVisible = modeState.mapMode === "sandbox";
    if (sceneRefs.mapGroup) {
      sceneRefs.mapGroup.visible = sandboxVisible;
    }
    if (sceneRefs.obstacleGroup) {
      sceneRefs.obstacleGroup.visible = sandboxVisible;
    }
    if (sceneRefs.tilesGroupOffset) {
      sceneRefs.tilesGroupOffset.visible = modeState.mapMode === "tiles3d";
    }
    scene.fog = sandboxVisible ? sceneRefs.defaultFog : null;
  }

  function updateTilesWorldMatrices() {
    sceneRefs.tilesGroupOffset?.updateMatrixWorld(true);
    scene.updateMatrixWorld(true);
  }

  function getTileCollisionMeshList() {
    updateTilesWorldMatrices();
    return [...modeState.tileCollisionMeshes].filter((mesh) => mesh.parent);
  }

  function getHitWorldNormalY(hit) {
    if (!hit.face) {
      return 0;
    }
    const normalMatrix = new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld);
    return hit.face.normal.clone().applyMatrix3(normalMatrix).normalize().y;
  }

  function getHitNormalDotUp(hit) {
    if (!hit.face) {
      return 0;
    }
    const normalMatrix = new THREE.Matrix3().getNormalMatrix(hit.object.matrixWorld);
    const worldNormal = hit.face.normal.clone().applyMatrix3(normalMatrix).normalize();
    return worldNormal.dot(getTilesUpAxis());
  }

  function findTilesSupportY(x, z, fallbackY = 0) {
    const meshes = getTileCollisionMeshList();
    if (meshes.length === 0) {
      return null;
    }

    tilesAnchoredLocalToWorld(x, fallbackY + 120, z, tilesRayOrigin);
    tilesRayDirection.copy(getTilesUpAxis()).multiplyScalar(-1);
    tilesFloorRaycaster.set(tilesRayOrigin, tilesRayDirection);
    tilesFloorRaycaster.far = 260;
    const hits = tilesFloorRaycaster.intersectObjects(meshes, false);
    const floorHits = hits
      .map((hit) => ({ hit, local: worldToTilesLocal(hit.point.clone()) }))
      .filter(({ hit, local }) => getHitNormalDotUp(hit) > 0.38 && local.y <= fallbackY + 1.2)
      .sort((a, b) => b.local.y - a.local.y);
    return floorHits.length > 0 ? floorHits[0].local.y : null;
  }

  function findTilesSpawnSurface() {
    const meshes = getTileCollisionMeshList();
    if (meshes.length === 0) {
      return {
        local: new THREE.Vector3(0, 0, 0),
        world: new THREE.Vector3(0, 0, 0),
      };
    }
    const worldBounds = new THREE.Box3();
    for (const mesh of meshes) {
      worldBounds.expandByObject(mesh);
    }
    if (worldBounds.isEmpty()) {
      return {
        local: new THREE.Vector3(0, 0, 0),
        world: new THREE.Vector3(0, 0, 0),
      };
    }
    const centerWorldX = (worldBounds.min.x + worldBounds.max.x) * 0.5;
    const centerWorldZ = (worldBounds.min.z + worldBounds.max.z) * 0.5;
    const sampleCols = 15;
    const sampleRows = 15;
    const startY = worldBounds.max.y + 120;
    let bestHit = null;
    let bestScore = Infinity;

    for (let ix = 0; ix < sampleCols; ix += 1) {
      for (let iz = 0; iz < sampleRows; iz += 1) {
        const tx = sampleCols === 1 ? 0.5 : ix / (sampleCols - 1);
        const tz = sampleRows === 1 ? 0.5 : iz / (sampleRows - 1);
        const worldX = THREE.MathUtils.lerp(worldBounds.min.x, worldBounds.max.x, tx);
        const worldZ = THREE.MathUtils.lerp(worldBounds.min.z, worldBounds.max.z, tz);
        tilesRayOrigin.set(worldX, startY, worldZ);
        tilesRayDirection.copy(getTilesUpAxis()).multiplyScalar(-1);
        tilesFloorRaycaster.set(tilesRayOrigin, tilesRayDirection);
        tilesFloorRaycaster.far = Math.max(300, worldBounds.max.y - worldBounds.min.y + 260);
        const hits = tilesFloorRaycaster.intersectObjects(meshes, false);
        const floorHit = hits
          .map((hit) => ({ hit, local: worldToTilesLocal(hit.point.clone()), world: hit.point.clone() }))
          .filter(({ hit }) => getHitNormalDotUp(hit) > 0.55)
          [0];
        if (!floorHit) {
          continue;
        }

        const dx = floorHit.hit.point.x - centerWorldX;
        const dz = floorHit.hit.point.z - centerWorldZ;
        const score = dx * dx + dz * dz;
        if (score < bestScore) {
          bestScore = score;
          bestHit = {
            local: floorHit.local.clone(),
            world: floorHit.world.clone(),
          };
        }
      }
    }

    if (bestHit) {
      return bestHit;
    }

    const world = new THREE.Vector3(centerWorldX, worldBounds.min.y, centerWorldZ);
    return {
      local: worldToTilesLocal(world.clone()),
      world,
    };
  }

  function findTilesSpawnFromVisibleBounds() {
    const meshes = getTileCollisionMeshList();
    if (meshes.length === 0) {
      return null;
    }

    const worldBounds = new THREE.Box3();
    for (const mesh of meshes) {
      worldBounds.expandByObject(mesh);
    }
    if (worldBounds.isEmpty()) {
      return null;
    }

    const centerX = (worldBounds.min.x + worldBounds.max.x) * 0.5;
    const centerZ = (worldBounds.min.z + worldBounds.max.z) * 0.5;
    tilesRayOrigin.set(centerX, worldBounds.max.y + 120, centerZ);
    tilesRayDirection.copy(getTilesUpAxis()).multiplyScalar(-1);
    tilesFloorRaycaster.set(tilesRayOrigin, tilesRayDirection);
    tilesFloorRaycaster.far = Math.max(300, worldBounds.max.y - worldBounds.min.y + 260);
    const hits = tilesFloorRaycaster.intersectObjects(meshes, false);
    const floorHit = hits
      .map((hit) => ({ hit, local: worldToTilesLocal(hit.point.clone()), world: hit.point.clone() }))
      .filter(({ hit }) => getHitNormalDotUp(hit) > 0.55)
      [0];

    return floorHit
      ? {
        local: floorHit.local.clone(),
        world: floorHit.world.clone(),
      }
      : null;
  }

  function isTilesHorizontalMotionBlocked(currentX, currentZ, nextX, nextZ, radius, supportY, minRayHeight = 0.18) {
    const meshes = getTileCollisionMeshList();
    if (meshes.length === 0) {
      return false;
    }

    tilesRayDirection.set(nextX - currentX, 0, nextZ - currentZ);
    const distance = tilesRayDirection.length();
    if (distance < 1e-5) {
      return false;
    }

    tilesRayDirection.normalize();
    const rayHeights = [minRayHeight, Math.max(minRayHeight + 0.18, 0.56), Math.max(minRayHeight + 0.36, 0.86)];
    for (const height of rayHeights) {
      tilesAnchoredLocalToWorld(currentX, supportY + height, currentZ, tilesRayOrigin);
      const worldStep = tilesAnchoredLocalToWorld(nextX, supportY + height, nextZ, new THREE.Vector3()).sub(tilesRayOrigin);
      const worldDistance = worldStep.length();
      tilesWallRaycaster.set(tilesRayOrigin, worldStep.normalize());
      tilesWallRaycaster.near = 0;
      tilesWallRaycaster.far = worldDistance + radius;
      const hits = tilesWallRaycaster.intersectObjects(meshes, false);
      if (hits.some((hit) => Math.abs(getHitNormalDotUp(hit)) < 0.72)) {
        return true;
      }
    }

    return false;
  }

  function resolveTilesMotion(nextX, nextZ, currentX, currentZ, currentSupportY, radius) {
    let resolvedX = currentX;
    let resolvedZ = currentZ;

    const candidateSupportX = findTilesSupportY(nextX, currentZ, currentSupportY);
    const xStepHeight = candidateSupportX === null ? Infinity : candidateSupportX - currentSupportY;
    const xRayMin = xStepHeight > 0 ? Math.min(0.88, xStepHeight + 0.16) : 0.18;
    if (
      candidateSupportX !== null &&
      xStepHeight <= tilesStepHeight &&
      !isTilesHorizontalMotionBlocked(currentX, currentZ, nextX, currentZ, radius, currentSupportY, xRayMin)
    ) {
      resolvedX = nextX;
    }

    const supportAfterX = findTilesSupportY(resolvedX, currentZ, currentSupportY);
    const baseSupportZ = supportAfterX ?? currentSupportY;
    const candidateSupportZ = findTilesSupportY(resolvedX, nextZ, baseSupportZ);
    const zStepHeight = candidateSupportZ === null ? Infinity : candidateSupportZ - baseSupportZ;
    const zRayMin = zStepHeight > 0 ? Math.min(0.88, zStepHeight + 0.16) : 0.18;
    if (
      supportAfterX !== null &&
      candidateSupportZ !== null &&
      zStepHeight <= tilesStepHeight &&
      !isTilesHorizontalMotionBlocked(resolvedX, currentZ, resolvedX, nextZ, radius, supportAfterX, zRayMin)
    ) {
      resolvedZ = nextZ;
    }

    const finalSupport = findTilesSupportY(resolvedX, resolvedZ, currentSupportY);

    return {
      x: finalSupport === null ? currentX : resolvedX,
      z: finalSupport === null ? currentZ : resolvedZ,
      supportY: finalSupport === null ? currentSupportY : finalSupport,
    };
  }

  function planTilesPath() {
    if (!modeState.tilesetLoaded || !modeState.tilesSpawned) {
      return { ok: false, reason: "tiles-not-ready" };
    }
    if (!plannerState.start || !plannerState.goal) {
      return { ok: false, reason: "missing-endpoints" };
    }

    const start = {
      x: plannerState.start.x,
      z: plannerState.start.z,
      y: plannerState.start.y ?? findTilesSupportY(plannerState.start.x, plannerState.start.z, motionState.supportY),
    };
    const goal = {
      x: plannerState.goal.x,
      z: plannerState.goal.z,
      y: plannerState.goal.y ?? findTilesSupportY(plannerState.goal.x, plannerState.goal.z, start.y ?? motionState.supportY),
    };
    if (start.y === null || goal.y === null) {
      return { ok: false, reason: "missing-surface" };
    }

    try {
      return planTilesRoute({
        start,
        goal,
        findSupportY: findTilesSupportY,
        isHorizontalBlocked: isTilesHorizontalMotionBlocked,
        robotRadius: activeRobot.collisionRadius,
        options: {
          maxUpStep: tilesStepHeight,
          maxDownStep: 0.95,
          clearance: activeRobot.collisionRadius + 0.12,
        },
      });
    } catch (error) {
      console.error(error);
      return { ok: false, reason: "planner-error" };
    }
  }

  function placeRobotOnTilesSpawn() {
    const spawnPoint = findTilesSpawnFromVisibleBounds() ?? findTilesSpawnSurface();
    if (!modeState.tilesSpawned && sceneRefs.tilesGroupOffset) {
      const recenterOffset = spawnPoint.world.clone().multiplyScalar(-1);
      sceneRefs.tilesGroupOffset.position.add(recenterOffset);
      modeState.tilesFrame?.origin?.add(recenterOffset);
      spawnPoint.world.add(recenterOffset);
      updateTilesWorldMatrices();
      requestTilesRefresh();
    }
    modeState.tilesRobotAnchorLocal = spawnPoint.local.clone();
    modeState.tilesRobotAnchorWorld = spawnPoint.world.clone();
    tilesMotionState.x = spawnPoint.local.x;
    tilesMotionState.z = spawnPoint.local.z;
    tilesMotionState.yaw = 0;
    tilesMotionState.jumpY = 0;
    tilesMotionState.velocityY = 0;
    tilesMotionState.isGrounded = true;
    tilesMotionState.supportY = spawnPoint.local.y;
    tilesMotionState.groundOffset = activeRobot.groundOffset;
    copyMotionState(motionState, tilesMotionState);
    syncActiveRobotTransform();
    activeRobot.root.visible = true;
    startMarker.visible = false;
    goalMarker.visible = false;
    routeGroup.visible = false;
    modeState.tilesSpawned = true;
  }

  function placeRobotOnTilesHit(hit) {
    if (!hit || getHitNormalDotUp(hit) <= 0.35) {
      updatePlannerStatus("Click an upward-facing tile surface to spawn the robot.");
      return false;
    }

    modeState.tilesSpawnArmed = false;
    const spawnWorld = hit.point.clone();
    const spawnLocal = worldToTilesLocal(spawnWorld.clone());
    modeState.tilesRobotAnchorLocal = spawnLocal.clone();
    modeState.tilesRobotAnchorWorld = spawnWorld.clone();
    tilesMotionState.x = spawnLocal.x;
    tilesMotionState.z = spawnLocal.z;
    tilesMotionState.yaw = 0;
    tilesMotionState.jumpY = 0;
    tilesMotionState.velocityY = 0;
    tilesMotionState.isGrounded = true;
    tilesMotionState.supportY = spawnLocal.y;
    tilesMotionState.groundOffset = activeRobot.groundOffset;
    copyMotionState(motionState, tilesMotionState);
    syncActiveRobotTransform();
    activeRobot.root.visible = true;
    startMarker.visible = false;
    goalMarker.visible = false;
    routeGroup.visible = false;
    modeState.tilesSpawned = true;
    syncTilesetControls();
    updatePlannerStatus("Robot spawned on the 3D Tiles surface. WASD now moves relative to the robot heading.");
    return true;
  }

  function scheduleRobotOnTilesSpawn() {
    if (modeState.tilesSpawned || tilesSpawnRequest !== null) {
      return;
    }

    tilesSpawnRequest = requestAnimationFrame(() => {
      tilesSpawnRequest = null;
      if (modeState.mapMode !== "tiles3d" || modeState.tilesSpawned || getTileCollisionMeshList().length === 0) {
        return;
      }
      placeRobotOnTilesSpawn();
    });
  }

  function requestTilesRefresh(frames = 24) {
    tilesRefreshFrames = Math.max(tilesRefreshFrames, frames);
  }

  function resetSandboxCamera() {
    cameraState.mode = "free";
    orbitState.azimuth = Math.PI * 0.84;
    orbitState.polar = 0.88;
    orbitState.distance = clamp(Math.max(cfg.map.width_m, cfg.map.height_m) * 0.78, 5.2, 16);
    orbitState.target.set(0, 0.28, 0);
    clampOrbitTarget();
    saveCurrentViewState("sandbox");
    syncCameraModeButton();
  }

  function disposeTilesetRenderer() {
    modeState.tilesRobotAnchorLocal = null;
    modeState.tilesRobotAnchorWorld = null;
    modeState.tilesSpawnArmed = false;
    if (tilesSpawnRequest !== null) {
      cancelAnimationFrame(tilesSpawnRequest);
      tilesSpawnRequest = null;
    }
    if (!sceneRefs.tilesRenderer) {
      return;
    }

    scene.remove(sceneRefs.tilesGroupOffset);
    sceneRefs.tilesRenderer.dispose?.();
    sceneRefs.tilesGroupOffset.clear();
    sceneRefs.tilesRenderer = null;
    sceneRefs.tilesGroupOffset = new THREE.Group();
    modeState.tileCollisionMeshes.clear();
    modeState.tilesetLoaded = false;
    modeState.tilesSpawned = false;
    modeState.tilesFrame = null;
  }

  async function loadTileset(url) {
    const trimmedUrl = url.trim();
    if (!trimmedUrl) {
      updatePlannerStatus("Enter a tileset.json path first.");
      return false;
    }

    disposeTilesetRenderer();
    updateLoadingState("Loading 3D Tiles...");
    updatePlannerStatus("Loading 3D Tiles tileset...");

    try {
      const regionFrame = await loadTilesetRegionFrame(trimmedUrl);
      const { TilesRenderer } = await import("3d-tiles-renderer");
      const [{ GLTFExtensionsPlugin }, { DRACOLoader }] = await Promise.all([
        import("https://esm.sh/3d-tiles-renderer@0.4.18/plugins?target=es2022&external=three"),
        import("three/examples/jsm/loaders/DRACOLoader.js"),
      ]);
      const tilesRenderer = new TilesRenderer(trimmedUrl);
      tilesRenderer.displayActiveTiles = true;
      tilesRenderer.errorTarget = 12;
      tilesRenderer.setCamera(camera);
      tilesRenderer.setResolutionFromRenderer(camera, renderer);
      const dracoLoader = new DRACOLoader();
      dracoLoader.setDecoderPath("https://unpkg.com/three@0.170.0/examples/jsm/libs/draco/gltf/");
      tilesRenderer.registerPlugin(
        new GLTFExtensionsPlugin({
          dracoLoader,
          rtc: true,
          metadata: true,
          plugins: [],
          ktxLoader: null,
          autoDispose: true,
        })
      );
      sceneRefs.tilesRenderer = tilesRenderer;
      sceneRefs.tilesGroupOffset = tilesRenderer.group;
      scene.add(sceneRefs.tilesGroupOffset);
      let didFrameTileset = false;
      let loadedModelCount = 0;
      const visibleTiles = new Set();

      const updateTilesDebugStatus = (prefix = "3D Tiles loaded") => {
        updatePlannerStatus(
          `${prefix}. models ${loadedModelCount}, visible ${visibleTiles.size}. Set Start and Goal on tile surfaces, then Plan for step-aware 3D routing.`
        );
      };

      const onTilesetReady = () => {
        if (didFrameTileset) {
          return;
        }
        const sphere = new THREE.Sphere();
        const hasBounds = tilesRenderer.getBoundingSphere(sphere);
        modeState.tilesFrame = regionFrame ?? {
          center: sphere.center.clone(),
          radius: sphere.radius,
          ...makeEcefAxes(sphere.center),
        };
        if (hasBounds && Number.isFinite(sphere.radius) && sphere.radius > 0) {
          tilesRenderer.group.position.copy(sphere.center).multiplyScalar(-1);
          tilesRenderer.group.quaternion.identity();
          modeState.tilesFrame.origin = regionFrame
            ? regionFrame.center.clone().sub(sphere.center)
            : new THREE.Vector3(0, 0, 0);
          sceneRefs.tilesGroupOffset.updateMatrixWorld(true);
          orbitState.target.set(0, 0, 0);
          orbitState.distance = clamp(sphere.radius * 2.2, 30, 12000);
          orbitState.azimuth = Math.PI * 0.72;
          orbitState.polar = 1.02;
          camera.near = 0.1;
          camera.far = Math.max(2500, sphere.radius * 50);
          camera.up.copy(getTilesUpAxis());
          camera.updateProjectionMatrix();
          clampOrbitTarget();
          requestTilesRefresh(36);
          didFrameTileset = true;
        }
        modeState.tilesetLoaded = true;
        updateLoadingState(`${activeRobot.label} + 3D Tiles View`);
        updateTilesDebugStatus("3D Tiles loaded. Click the tiles surface to spawn the robot");
      };

      tilesRenderer.addEventListener("load-tileset", onTilesetReady);
      tilesRenderer.addEventListener("load-tile-set", onTilesetReady);
      tilesRenderer.addEventListener("tiles-load-start", () => {
        updatePlannerStatus("3D Tiles download started...");
      });
      tilesRenderer.addEventListener("tiles-load-end", () => {
        if (modeState.tilesetLoaded) {
          updateTilesDebugStatus("3D Tiles visible");
        } else {
          updatePlannerStatus("3D Tiles downloads finished, but no centered view was confirmed yet.");
        }
      });
      tilesRenderer.addEventListener("load-error", (event) => {
        const message = event?.error?.message ?? event?.message ?? "Unknown 3D Tiles load error";
        updateLoadingState("3D Tiles load error");
        updatePlannerStatus(`3D Tiles load error: ${message}`);
      });

      tilesRenderer.addEventListener("load-model", ({ scene: tileScene }) => {
        loadedModelCount += 1;
        tileScene.traverse((node) => {
          if (node.isMesh) {
            node.castShadow = true;
            node.receiveShadow = true;
            modeState.tileCollisionMeshes.add(node);
          }
        });
        if (modeState.tilesetLoaded) {
          updateTilesDebugStatus("3D Tiles model loaded");
        }
      });
      tilesRenderer.addEventListener("tile-visibility-change", (event) => {
        if (event.visible) {
          visibleTiles.add(event.tile);
        } else {
          visibleTiles.delete(event.tile);
        }
        if (modeState.tilesetLoaded) {
          updateTilesDebugStatus("3D Tiles visible");
        }
      });

      modeState.tilesetUrl = trimmedUrl;
      saveStoredTilesetUrl(trimmedUrl);
      modeState.mapMode = "tiles3d";
      modeState.tilesetPanelOpen = true;
      syncMapModeSceneVisibility();
      syncTilesetControls();
      return true;
    } catch (error) {
      disposeTilesetRenderer();
      modeState.mapMode = "sandbox";
      modeState.tilesetLoaded = false;
      syncMapModeSceneVisibility();
      syncTilesetControls();
      updateLoadingState("3D Tiles load failed");
      updatePlannerStatus(`Failed to load 3D Tiles: ${error.message ?? error}`);
      return false;
    }
  }

  async function setMapMode(mode, options = {}) {
    saveCurrentModeMotionState();
    saveCurrentViewState(modeState.mapMode);
    modeState.mapMode = mode;

    if (mode === "tiles3d") {
      loadModeMotionState("tiles3d");
      loadViewState("tiles3d");
      if (cameraState.mode === "follow") {
        setCameraMode("free");
      }
      camera.up.copy(getTilesUpAxis());
      const shouldLoad = options.forceReload || !sceneRefs.tilesRenderer || !modeState.tilesetLoaded;
      if (shouldLoad) {
        const url = document.getElementById("tileset-url-input")?.value ?? modeState.tilesetUrl;
        return await loadTileset(url);
      } else {
        modeState.tilesetPanelOpen = true;
        syncMapModeSceneVisibility();
        syncTilesetControls();
        updatePlannerStatus("3D Tiles mode active. Set Start and Goal on tile surfaces, then Plan.");
        return true;
      }
    } else {
      disposeTilesetRenderer();
      loadModeMotionState("sandbox");
      loadViewState("sandbox");
      modeState.tilesetPanelOpen = false;
      syncMapModeSceneVisibility();
      syncTilesetControls();
      camera.up.copy(upAxis);
      camera.near = 0.1;
      camera.far = 180;
      camera.updateProjectionMatrix();
      updateLoadingState(activeRobot.type === "turtlebot3" ? "Wheel Robot + Navigation View" : "Legged Robot + Navigation View");
      updatePlannerStatus("Sandbox map mode active.");
      return true;
    }
  }

  function updateEditorStatus(text) {
    const node = document.querySelector('[data-role="editor-status"]');
    if (node) {
      node.textContent = text;
    }
  }

  function clampOrbitTarget() {
    if (modeState.mapMode === "tiles3d") {
      return;
    }
    orbitState.target.x = clamp(orbitState.target.x, -cfg.map.width_m * 0.85, cfg.map.width_m * 0.85);
    orbitState.target.z = clamp(orbitState.target.z, -cfg.map.height_m * 0.85, cfg.map.height_m * 0.85);
    orbitState.target.y = clamp(orbitState.target.y, 0.05, 3.5);
  }

  function syncCameraModeButton() {
    const button = document.getElementById("camera-mode-btn");
    if (!button) {
      return;
    }
    button.innerHTML =
      cameraState.mode === "follow"
        ? "<strong>Camera</strong><span>Follow view</span>"
        : "<strong>Camera</strong><span>Free view</span>";
    button.classList.toggle("is-active", cameraState.mode === "free");
  }

  function setCameraMode(mode) {
    cameraState.mode = mode;
    if (mode === "free") {
      if (modeState.mapMode === "tiles3d") {
        focusOrbitTargetOnRobot();
      } else {
        orbitState.target.set(motionState.x, 0.32, motionState.z);
        clampOrbitTarget();
      }
    } else if (modeState.mapMode === "tiles3d") {
      orbitState.distance = clamp(orbitState.distance, 14, 32);
      orbitState.azimuth = Math.PI;
      orbitState.polar = 1.02;
      focusOrbitTargetOnRobot(0.18);
    }
    saveCurrentViewState(modeState.mapMode);
    syncCameraModeButton();
  }

  function syncGuideVisibility() {
    const guide = document.querySelector('[data-role="control-guide"]');
    if (guide) {
      guide.classList.toggle("is-hidden", !guideState.visible);
      guide.setAttribute("aria-hidden", guideState.visible ? "false" : "true");
    }
    const button = document.getElementById("guide-toggle-btn");
    if (button) {
      button.innerHTML = guideState.visible
        ? "<strong>Guide</strong><span>Hide help panel</span>"
        : "<strong>Guide</strong><span>Show help with E</span>";
      button.classList.toggle("is-active", guideState.visible);
    }
  }

  function toggleGuideVisibility() {
    guideState.visible = !guideState.visible;
    syncGuideVisibility();
  }

  function hydrateHudButtons() {
    const controlsNode = document.querySelector(".controls");
    if (controlsNode) {
      controlsNode.remove();
    }

    const buttonContent = {
      "camera-mode-btn": ["Camera", "Follow or free view"],
      "guide-toggle-btn": ["Guide", "Toggle help with E"],
      "set-start-btn": ["Start", "Place route start"],
      "set-goal-btn": ["Goal", "Place route goal"],
      "plan-path-btn": ["Plan", "Build shortest path"],
      "auto-nav-btn": ["Auto", "Start or stop tracking"],
      "viewer-page-btn": ["Viewer", "Return to sandbox view"],
      "map-edit-btn": ["Map Edit", "Open 3D editor"],
      "tiles-page-btn": ["3D City", "Open PLATEAU viewer"],
      "tiles-mode-btn": ["3D Tiles", "Open city tiles mode"],
      "tileset-load-btn": ["Load Tileset", "Read tileset.json"],
      "tileset-sandbox-btn": ["Sandbox", "Return to flat map"],
      "editor-select-btn": ["Select", "Pick and move"],
      "editor-add-btn": ["Add", "Place obstacle"],
      "editor-delete-btn": ["Delete", "Remove obstacle"],
      "editor-spawn-btn": ["Spawn", "Set robot start"],
      "editor-close-btn": ["Close", "Return to viewer"],
      "editor-apply-btn": ["Apply", "Save map changes"],
    };

    for (const [id, [title, subtitle]] of Object.entries(buttonContent)) {
      const button = document.getElementById(id);
      if (!button) {
        continue;
      }
      button.classList.add("action-button");
      button.innerHTML = `<strong>${title}</strong><span>${subtitle}</span>`;
    }

    const startButton = document.getElementById("set-start-btn");
    if (startButton) {
      startButton.classList.add("with-icon");
      startButton.innerHTML =
        '<div class="button-copy"><strong>Start</strong><span>Place route start</span></div><span class="button-icon start" aria-hidden="true"></span>';
    }

    const goalButton = document.getElementById("set-goal-btn");
    if (goalButton) {
      goalButton.classList.add("with-icon");
      goalButton.innerHTML =
        '<div class="button-copy"><strong>Goal</strong><span>Place route goal</span></div><span class="button-icon goal" aria-hidden="true"></span>';
    }

    document.getElementById("set-start-btn")?.classList.add("route-button");
    document.getElementById("set-goal-btn")?.classList.add("route-button");
    document.getElementById("plan-path-btn")?.classList.add("route-button");
    document.getElementById("auto-nav-btn")?.classList.add("route-button");
    if (appMode !== "tiles") {
      document.getElementById("map-edit-btn")?.classList.add("accent-button");
    }
  }

  function hydrateGuidePanel() {
    const guide = document.querySelector('[data-role="control-guide"]');
    if (!guide) {
      return;
    }

    const viewerItems = [
      [["W"], "Move forward"],
      [["A"], "Move left"],
      [["S"], "Move backward"],
      [["D"], "Move right"],
      [["Left", "Right"], "Yaw turn"],
      [["LMB"], "Orbit around the center reticle"],
      [["Double", "Click"], "Move orbit center to the clicked surface"],
      [["RMB"], "Pan camera in free view"],
      [["Wheel"], "Zoom camera"],
      [["F"], "Sprint forward"],
      [["Shift"], "Sneak"],
      [["Space"], "Jump"],
      [["Start"], "Place route start"],
      [["Goal"], "Place route goal"],
      [["Plan"], "Build shortest path"],
      [["Auto"], "Run autonomous path tracking"],
      [["Map Edit"], "Open the 3D map editor"],
      [["E"], "Show or hide this guide"],
    ];
    const editorItems = [
      [["LMB"], "Select or place objects"],
      [["Drag"], "Move the selected box on the map plane"],
      [["LMB", "Empty"], "Orbit around the center reticle"],
      [["Double", "Click"], "Move orbit center to the clicked surface"],
      [["RMB"], "Pan the view"],
      [["Wheel"], "Zoom in or out"],
      [["Apply"], "Save the edited map"],
      [["Close"], "Return to the robot viewer"],
      [["E"], "Show or hide this guide"],
    ];
    const items = appMode === "editor" ? editorItems : viewerItems;

    guide.innerHTML = [
      `<h3 class="guide-title">Controls</h3>`,
      ...items.map(
        ([keys, text]) => `
      <div class="guide-item">
        <div class="guide-keys">${keys
          .map((key) => `<span class="guide-key">${key}</span>`)
          .join("")}</div>
        <span>${text}</span>
      </div>`
      ),
    ].join("");
  }

  function syncEditorForm() {
    document.getElementById("editor-map-width").value = cfg.map.width_m.toFixed(1);
    document.getElementById("editor-map-height").value = cfg.map.height_m.toFixed(1);
    document.getElementById("editor-obstacle-width").value = "0.8";
    document.getElementById("editor-obstacle-depth").value = "0.8";
    document.getElementById("editor-obstacle-height").value = "0.6";
    document.getElementById("editor-spawn-yaw").value = String(cfg.go2_pose.yaw_deg ?? 0);
    syncSelectedObstacleInputs();
  }

  function drawEditorView() {
    const canvas = document.getElementById("editor-canvas");
    if (!canvas) {
      return;
    }
    const ctx = canvas.getContext("2d");
    const width = canvas.width;
    const height = canvas.height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#07101a";
    ctx.fillRect(0, 0, width, height);

    const pad = 24;
    const scale = Math.min(
      (width - pad * 2) / cfg.map.width_m,
      (height - pad * 2) / cfg.map.height_m
    );
    const mapLeft = pad;
    const mapTop = pad;

    function toCanvasMap(point) {
      return {
        x: mapLeft + point.x * scale,
        y: mapTop + point.y * scale,
      };
    }

    ctx.strokeStyle = "rgba(126, 208, 255, 0.16)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= cfg.map.width_m; x += 1) {
      const xPos = mapLeft + x * scale;
      ctx.beginPath();
      ctx.moveTo(xPos, mapTop);
      ctx.lineTo(xPos, mapTop + cfg.map.height_m * scale);
      ctx.stroke();
    }
    for (let y = 0; y <= cfg.map.height_m; y += 1) {
      const yPos = mapTop + y * scale;
      ctx.beginPath();
      ctx.moveTo(mapLeft, yPos);
      ctx.lineTo(mapLeft + cfg.map.width_m * scale, yPos);
      ctx.stroke();
    }

    ctx.fillStyle = "rgba(159, 215, 255, 0.75)";
    for (const obstacle of mapState.obstacles) {
      const point = toCanvasMap({ x: obstacle.x, y: obstacle.y });
      ctx.fillRect(point.x, point.y, obstacle.w * scale, obstacle.d * scale);
    }

    const spawn = toCanvasMap({
      x: cfg.go2_pose.x_m,
      y: cfg.go2_pose.y_m,
    });
    ctx.fillStyle = "#4de18b";
    ctx.beginPath();
    ctx.arc(spawn.x, spawn.y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#4de18b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(spawn.x, spawn.y);
    ctx.lineTo(
      spawn.x + Math.cos((-cfg.go2_pose.yaw_deg * Math.PI) / 180) * 16,
      spawn.y + Math.sin((-cfg.go2_pose.yaw_deg * Math.PI) / 180) * 16
    );
    ctx.stroke();
  }

  function resetEditorCamera() {
    orbitState.azimuth = Math.PI * 0.84;
    orbitState.polar = 0.88;
    orbitState.distance = clamp(Math.max(cfg.map.width_m, cfg.map.height_m) * 0.78, 5.2, 16);
    orbitState.target.set(0, 0.28, 0);
    clampOrbitTarget();
  }

  function clampEditorCameraTarget() {
    clampOrbitTarget();
  }

  function showEditorScreen(visible) {
    const screen = document.getElementById("editor-screen");
    screen.classList.toggle("is-visible", visible);
    screen.setAttribute("aria-hidden", visible ? "false" : "true");
    editorState.isOpen = visible;
    if (visible) {
      resetEditorCamera();
      syncEditorForm();
      drawEditorView();
      updateEditorStatus("Left drag orbits on empty space. Right drag pans. Click a box to select or move it.");
      setEditorMode("select");
      clearPathState();
      rebuildSelectionOverlay();
    }
  }

  function editorCanvasToMapPoint(event) {
    const canvas = document.getElementById("editor-canvas");
    const rect = canvas.getBoundingClientRect();
    const pad = 24;
    const scale = Math.min(
      (canvas.width - pad * 2) / cfg.map.width_m,
      (canvas.height - pad * 2) / cfg.map.height_m
    );
    const x = clamp((event.clientX - rect.left - pad) / rect.width * canvas.width / scale, 0, cfg.map.width_m);
    const y = clamp((event.clientY - rect.top - pad) / rect.height * canvas.height / scale, 0, cfg.map.height_m);
    return { x, y };
  }

  function setEditorMode(mode) {
    editorState.mode = mode;
    editorState.armedObstacleId = null;
    editorState.armedPointerId = null;
    editorState.draggingObstacleId = null;
    editorState.dragPointerId = null;
    setButtonActive("editor-select-btn", mode === "select");
    setButtonActive("editor-add-btn", mode === "add");
    setButtonActive("editor-delete-btn", mode === "delete");
    setButtonActive("editor-spawn-btn", mode === "spawn");
  }

  function applyMapChangesFromEditor() {
    const nextWidth = Number(document.getElementById("editor-map-width").value);
    const nextHeight = Number(document.getElementById("editor-map-height").value);
    const nextYaw = Number(document.getElementById("editor-spawn-yaw").value);
    cfg.map.width_m = clamp(Number.isFinite(nextWidth) ? nextWidth : cfg.map.width_m, 4, 30);
    cfg.map.height_m = clamp(Number.isFinite(nextHeight) ? nextHeight : cfg.map.height_m, 4, 30);
    cfg.go2_pose.yaw_deg = clamp(Number.isFinite(nextYaw) ? nextYaw : cfg.go2_pose.yaw_deg, -180, 180);
    mapState.obstacles = mapState.obstacles.map((obstacle) => ({
      ...obstacle,
      x: clamp(obstacle.x, 0, Math.max(0, cfg.map.width_m - obstacle.w)),
      y: clamp(obstacle.y, 0, Math.max(0, cfg.map.height_m - obstacle.d)),
      elevation: clamp(obstacle.elevation ?? 0, 0, 4),
    }));
    cfg.go2_pose.x_m = clamp(cfg.go2_pose.x_m, 0, cfg.map.width_m);
    cfg.go2_pose.y_m = clamp(cfg.go2_pose.y_m, 0, cfg.map.height_m);

    refreshSceneGeometry();
    clampEditorCameraTarget();
    resetRobotsToConfigPose();
    setPlannerMode(null);
    updateHud(cfg, activeRobot.label);
    drawEditorView();
    saveStoredMapState(cfg, mapState);
    selectObstacle(editorState.selectedObstacleId);
    updateEditorStatus("Map changes applied.");
  }

  function getSupportHeightAt(point) {
    return evaluateSupportSurface(
      point.x,
      point.z,
      0.02,
      0,
      Number.POSITIVE_INFINITY,
      obstacleBounds
    ).supportY;
  }

  function worldToMapPoint(position) {
    return { x: position.x, z: position.z };
  }

  function mapCoordsToWorldPoint(x, y) {
    return mapToWorld(x, y, cfg.map.width_m, cfg.map.height_m);
  }

  function worldToMapCoords(x, z) {
    return {
      x: x + cfg.map.width_m / 2,
      y: z + cfg.map.height_m / 2,
    };
  }

  function getObstacleById(id) {
    return mapState.obstacles.find((obstacle) => obstacle.id === id) ?? null;
  }

  function rebuildSelectionOverlay() {
    selectionOverlayGroup.clear();
    selectionHandleMeshes.length = 0;

    if (!editorState.isOpen || !editorState.selectedObstacleId) {
      return;
    }

    const obstacle = getObstacleById(editorState.selectedObstacleId);
    if (!obstacle) {
      return;
    }

    const footprint = mapToWorld(
      obstacle.x + obstacle.w / 2,
      obstacle.y + obstacle.d / 2,
      cfg.map.width_m,
      cfg.map.height_m
    );
    const centerY = (obstacle.elevation ?? 0) + obstacle.h / 2;

    const outline = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(obstacle.w, obstacle.h, obstacle.d)),
      new THREE.LineBasicMaterial({ color: 0xf6ff7a, transparent: true, opacity: 0.95 })
    );
    outline.position.set(footprint.x, centerY, footprint.z);
    selectionOverlayGroup.add(outline);

    const highlightShell = new THREE.Mesh(
      new THREE.BoxGeometry(obstacle.w * 1.01, obstacle.h * 1.01, obstacle.d * 1.01),
      new THREE.MeshBasicMaterial({
        color: 0xefff7a,
        transparent: true,
        opacity: 0.08,
        depthWrite: false,
      })
    );
    highlightShell.position.copy(outline.position);
    selectionOverlayGroup.add(highlightShell);

    const handleSpecs = [
      {
        axis: "x",
        color: 0x41e6ff,
        position: [footprint.x + obstacle.w / 2, centerY, footprint.z],
      },
      {
        axis: "z",
        color: 0xff9a3d,
        position: [footprint.x, centerY, footprint.z + obstacle.d / 2],
      },
      {
        axis: "y",
        color: 0xff5fc8,
        position: [footprint.x, (obstacle.elevation ?? 0) + obstacle.h, footprint.z],
      },
    ];

    for (const spec of handleSpecs) {
      const handle = new THREE.Mesh(
        new THREE.SphereGeometry(0.085, 18, 18),
        new THREE.MeshStandardMaterial({
          color: spec.color,
          emissive: spec.color,
          emissiveIntensity: 0.55,
          roughness: 0.28,
          metalness: 0.08,
        })
      );
      handle.position.set(...spec.position);
      handle.userData.handleAxis = spec.axis;
      handle.userData.obstacleId = obstacle.id;
      selectionOverlayGroup.add(handle);
      selectionHandleMeshes.push(handle);
    }
  }

  function syncSelectedObstacleInputs() {
    const obstacle = getObstacleById(editorState.selectedObstacleId);
    const fields = [
      "editor-box-x",
      "editor-box-y",
      "editor-box-elevation",
      "editor-box-height",
      "editor-box-width",
      "editor-box-depth",
    ];
    if (!obstacle) {
      for (const id of fields) {
        const input = document.getElementById(id);
        if (input) {
          input.value = "";
        }
      }
      return;
    }
    document.getElementById("editor-box-x").value = obstacle.x.toFixed(2);
    document.getElementById("editor-box-y").value = obstacle.y.toFixed(2);
    document.getElementById("editor-box-elevation").value = (obstacle.elevation ?? 0).toFixed(2);
    document.getElementById("editor-box-height").value = obstacle.h.toFixed(2);
    document.getElementById("editor-box-width").value = obstacle.w.toFixed(2);
    document.getElementById("editor-box-depth").value = obstacle.d.toFixed(2);
  }

  function selectObstacle(id) {
    editorState.selectedObstacleId = id;
    for (const mesh of sceneRefs.obstacleMeshes ?? []) {
      const isSelected = mesh.userData.obstacleId === id;
      mesh.material.emissive.setHex(isSelected ? 0xd7ff5a : 0x000000);
      mesh.material.emissiveIntensity = isSelected ? 0.68 : 0;
      mesh.material.color.setHex(isSelected ? 0xc8f4ff : 0x9fd7ff);
    }
    rebuildSelectionOverlay();
    syncSelectedObstacleInputs();
  }

  function clearPlannedPath() {
    clearPathState();
  }

  function setStartToRobotPosition() {
    if (modeState.mapMode === "tiles3d" && !modeState.tilesSpawned) {
      updatePlannerStatus("Load 3D Tiles and spawn the robot before setting the start point.");
      return;
    }
    const startPoint = modeState.mapMode === "tiles3d"
      ? { x: motionState.x, z: motionState.z, y: motionState.supportY }
      : worldToMapPoint(activeRobot.root.position);
    plannerState.start = startPoint;
    setMarkerPosition(startMarker, startPoint, motionState.supportY);
    clearPathState();
    setPlannerMode("start");
    updatePlannerStatus(
      modeState.mapMode === "tiles3d"
        ? "Start point set to robot position. Click a tile surface to move it."
        : "Start point set to robot position. Click the map to move it."
    );
  }

  function planCurrentPath() {
    if (!plannerState.start || !plannerState.goal) {
      updatePlannerStatus("Set both start and goal first.");
      return;
    }

    if (modeState.mapMode === "tiles3d") {
      const planResult = planTilesPath();
      if (!planResult.ok) {
        plannerState.path = null;
        plannerState.polylineMetrics = null;
        plannerState.smoothPath = null;
        plannerState.smoothPathMetrics = null;
        plannerState.debugData = null;
        plannerState.autoActive = false;
        plannerState.followerKey = null;
        plannerState.followerState = null;
        clearRouteRender();
        showPlannerDebugPanel(false);
        if (planResult.debugData) {
          drawTilesRouteDebugView(planResult.debugData, "Failed");
        } else {
          showTilesRouteDebugPanel(false);
        }
        const reasonText = {
          "tiles-not-ready": "Load 3D Tiles and spawn the robot before planning.",
          "missing-endpoints": "Set both start and goal first.",
          "missing-surface": "Start or goal is not on a walkable tile surface.",
          "endpoint-clearance": "Start or goal is too close to a wall, column, or edge for the robot footprint.",
          "search-budget": "3D Tiles path search stopped before finding a route. Try closer points or a clearer area.",
          "planner-error": "3D Tiles planner failed. Try reloading the tileset or choosing closer route points.",
          "no-tiles-path": "No step-aware 3D Tiles path found in the local search area.",
        };
        updatePlannerStatus(reasonText[planResult.reason] ?? "No 3D Tiles path found.");
        return;
      }

      plannerState.path = planResult.path;
      plannerState.polylineMetrics = planResult.polylineMetrics;
      plannerState.smoothPath = planResult.smoothPath;
      plannerState.smoothPathMetrics = planResult.smoothPathMetrics;
      plannerState.debugData = null;
      plannerState.autoActive = false;
      plannerState.followerKey = null;
      plannerState.followerState = null;
      clearRouteRender();
      routeGroup.add(buildTilesPathRenderables(plannerState.path));
      routeGroup.visible = true;
      showPlannerDebugPanel(false);
      drawTilesRouteDebugView(planResult.debugData, "Solved");
      updatePlannerStatus(
        `3D path planned. ${plannerState.path.length} waypoints / ${planResult.totalLength.toFixed(2)}m / ${planResult.visitedCount ?? 0} searched.`
      );
      return;
    }

    const plannerModule = plannerModules[plannerState.plannerKey];
    const planResult = plannerModule.plan({
      start: plannerState.start,
      goal: plannerState.goal,
      obstacleBounds,
      mapWidth: cfg.map.width_m,
      mapHeight: cfg.map.height_m,
      clearance: robotCollisionRadius + 0.03,
    });

    if (!planResult.ok) {
      plannerState.path = null;
      plannerState.polylineMetrics = null;
      plannerState.smoothPath = null;
      plannerState.smoothPathMetrics = null;
      plannerState.debugData = planResult.debugData ?? null;
      plannerState.autoActive = false;
      plannerState.followerKey = null;
      plannerState.followerState = null;
      clearRouteRender();
      if (planResult.debugData) {
        drawPlannerDebugView(planResult.debugData, cfg, activeRobot.label);
      } else {
        showPlannerDebugPanel(false);
      }
      updatePlannerStatus(
        planResult.reason === "blocked-endpoint"
          ? "Start or goal is inside an obstacle margin."
          : "No collision-free path found."
      );
      return;
    }

    plannerState.path = planResult.path;
    plannerState.polylineMetrics = planResult.polylineMetrics;
    plannerState.smoothPath = planResult.smoothPath;
    plannerState.smoothPathMetrics = planResult.smoothPathMetrics;
    plannerState.debugData = planResult.debugData;
    plannerState.autoActive = false;
    plannerState.followerKey = null;
    plannerState.followerState = null;
    clearRouteRender();
    routeGroup.add(buildPathRenderables(plannerState.path, activeRobot.type));
    drawPlannerDebugView(plannerState.debugData, cfg, activeRobot.label);
    updatePlannerStatus(`Path planned. ${plannerState.path.length} nodes / ${planResult.totalLength.toFixed(2)}m`);
  }

  function beginAutoNav() {
    if (!plannerState.path || plannerState.path.length < 2) {
      updatePlannerStatus("Plan a path before starting auto nav.");
      return;
    }

    plannerState.autoActive = true;
    autoCommandState.forward = 0;
    autoCommandState.lateral = 0;
    autoCommandState.turn = 0;
    autoCommandState.speed = 0;
    showPlannerDebugPanel(false);
    const followerKey = activeRobot.type === "go2" ? "go2Polyline" : "turtlebotSmooth";
    const followerModule = followerModules[followerKey];

    if (followerKey === "turtlebotSmooth" && (!plannerState.smoothPath || !plannerState.smoothPathMetrics)) {
      updatePlannerStatus("Plan a smooth path before starting auto nav.");
      plannerState.autoActive = false;
      return;
    }

    plannerState.followerKey = followerKey;
    plannerState.followerState = followerModule.begin({
      plannerState,
      motionState,
      projectPointOntoPath,
    });
    updatePlannerStatus("Auto navigation running.");
    document.getElementById("auto-nav-btn").textContent = "Stop Auto";
    document.getElementById("auto-nav-btn").classList.add("is-active");
  }

  function stopAutoNav(statusText) {
    plannerState.autoActive = false;
    plannerState.followerKey = null;
    plannerState.followerState = null;
    autoCommandState.forward = 0;
    autoCommandState.lateral = 0;
    autoCommandState.turn = 0;
    autoCommandState.speed = 0;
    document.getElementById("auto-nav-btn").textContent = "Start Auto";
    document.getElementById("auto-nav-btn").classList.remove("is-active");
    if (statusText) {
      updatePlannerStatus(statusText);
    }
  }

  function syncPoseToHud() {
    if (modeState.mapMode === "tiles3d" && modeState.tilesRobotAnchorLocal) {
      cfg.go2_pose.x_m = Number((motionState.x - modeState.tilesRobotAnchorLocal.x).toFixed(2));
      cfg.go2_pose.y_m = Number((motionState.z - modeState.tilesRobotAnchorLocal.z).toFixed(2));
    } else {
      cfg.go2_pose.x_m = Number((motionState.x + cfg.map.width_m / 2).toFixed(2));
      cfg.go2_pose.y_m = Number((motionState.z + cfg.map.height_m / 2).toFixed(2));
    }
    cfg.go2_pose.yaw_deg = Math.round((-motionState.yaw * 180) / Math.PI);
    updateHud(cfg, activeRobot.label);
  }

  function onKeyChange(event, isPressed) {
    const key = event.code;
    const tagName = event.target?.tagName ?? "";
    const isTypingTarget =
      tagName === "INPUT" || tagName === "TEXTAREA" || tagName === "SELECT" || event.target?.isContentEditable;

    if (key === "KeyE" && isPressed && !event.repeat && !isTypingTarget) {
      event.preventDefault();
      toggleGuideVisibility();
      return;
    }

    if (
      [
        "KeyW",
        "KeyA",
        "KeyS",
        "KeyD",
        "KeyF",
        "ArrowLeft",
        "ArrowRight",
        "Space",
        "ShiftLeft",
        "ShiftRight",
      ].includes(key)
    ) {
      event.preventDefault();
    }

    if (isTypingTarget) {
      return;
    }

    if (key === "KeyW") inputState.forward = isPressed;
    if (key === "KeyS") inputState.backward = isPressed;
    if (key === "KeyA") inputState.left = isPressed;
    if (key === "KeyD") inputState.right = isPressed;
    if (key === "KeyF") inputState.sprint = isPressed;
    if (key === "ArrowLeft") inputState.turnLeft = isPressed;
    if (key === "ArrowRight") inputState.turnRight = isPressed;
    if (key === "ShiftLeft" || key === "ShiftRight") inputState.sneak = isPressed;

    if (key === "Space" && isPressed && motionState.isGrounded && activeRobot.canJump) {
      motionState.velocityY = jumpSpeed;
      motionState.isGrounded = false;
    }
  }

  window.addEventListener("keydown", (event) => onKeyChange(event, true));
  window.addEventListener("keyup", (event) => onKeyChange(event, false));

  function setPointerFromEvent(event) {
    const rect = renderer.domElement.getBoundingClientRect();
    pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
  }

  function focusOrbitTargetFromPointer(event) {
    if (cameraState.mode !== "free" || editorState.isOpen || plannerState.mode || modeState.tilesSpawnArmed) {
      return;
    }
    setPointerFromEvent(event);

    // Double-click recenters the free camera orbit on an actual map surface.
    if (modeState.mapMode === "tiles3d") {
      const tileHits = raycaster
        .intersectObjects(getTileCollisionMeshList(), false)
        .filter((hit) => getHitNormalDotUp(hit) > 0.18);
      if (tileHits.length === 0) {
        return;
      }
      orbitState.target.copy(tileHits[0].point);
    } else if (raycaster.ray.intersectPlane(floorPlane, planeHit)) {
      orbitState.target.copy(planeHit);
    } else {
      return;
    }

    clampOrbitTarget();
    saveCurrentViewState(modeState.mapMode);
    updateOrbitReticle();
  }

  function intersectGroundPlaneAtY(y) {
    const dragPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -y);
    return raycaster.ray.intersectPlane(dragPlane, planeHit);
  }

  function beginCameraDrag(event, dragMode) {
    orbitState.dragging = true;
    orbitState.dragMode = dragMode;
    orbitState.pointerId = event.pointerId;
    orbitState.lastX = event.clientX;
    orbitState.lastY = event.clientY;
    if (dragMode === "pan") {
      orbitState.panPlaneY = 0;
      if (modeState.mapMode === "tiles3d") {
        camera.updateMatrixWorld();
        orbitState.panRight.setFromMatrixColumn(camera.matrixWorld, 0).normalize();
        orbitState.panUp.setFromMatrixColumn(camera.matrixWorld, 1).normalize();
      }
    }
    renderer.domElement.setPointerCapture(event.pointerId);
  }

  function onPointerDown(event) {
    setPointerFromEvent(event);

    if (modeState.mapMode === "tiles3d" && !editorState.isOpen) {
      if (event.button === 2) {
        beginCameraDrag(event, "pan");
        return;
      }

      if (event.button === 0 && plannerState.mode) {
        const tileHits = raycaster
          .intersectObjects(getTileCollisionMeshList(), false)
          .filter((hit) => getHitNormalDotUp(hit) > 0.35);
        if (tileHits.length === 0) {
          updatePlannerStatus("Click an upward-facing tile surface to place the route point.");
          return;
        }
        const hit = tileHits[0];
        const local = worldToTilesLocal(hit.point.clone());
        const point = { x: local.x, z: local.z, y: local.y };
        if (plannerState.mode === "start") {
          plannerState.start = point;
          setMarkerPosition(startMarker, point, point.y);
          updatePlannerStatus("3D Tiles start point set.");
        } else {
          plannerState.goal = point;
          setMarkerPosition(goalMarker, point, point.y);
          updatePlannerStatus("3D Tiles goal point set.");
        }
        clearPlannedPath();
        setPlannerMode(null);
        return;
      }

      if (event.button === 0 && modeState.tilesSpawnArmed) {
        const tileHits = raycaster
          .intersectObjects(getTileCollisionMeshList(), false)
          .filter((hit) => getHitNormalDotUp(hit) > 0.35);
        if (tileHits.length > 0 && placeRobotOnTilesHit(tileHits[0])) {
          return;
        }
      }

      if (event.button === 0) {
        beginCameraDrag(event, "orbit");
      }
      return;
    }

    if (editorState.isOpen) {
      if (event.button === 2) {
        beginCameraDrag(event, "pan");
        return;
      }

      const handleHits = raycaster.intersectObjects(selectionHandleMeshes, false);
      if (editorState.mode === "select" && handleHits.length > 0) {
        const handle = handleHits[0].object;
        const obstacle = getObstacleById(handle.userData.obstacleId);
        if (obstacle) {
          selectObstacle(obstacle.id);
          editorState.resizingHandleAxis = handle.userData.handleAxis;
          editorState.resizePointerId = event.pointerId;
          editorState.resizeObstacleId = obstacle.id;
          editorState.resizePlaneY = (obstacle.elevation ?? 0) + obstacle.h / 2;
          editorState.resizeStartClientY = event.clientY;
          editorState.resizeBaseHeight = obstacle.h;
          renderer.domElement.setPointerCapture(event.pointerId);
          updateEditorStatus(
            handle.userData.handleAxis === "x"
              ? "Drag the cyan handle to change width."
              : handle.userData.handleAxis === "z"
                ? "Drag the orange handle to change depth."
                : "Drag the pink handle to change height."
          );
          return;
        }
      }

      const hits = raycaster.intersectObjects(sceneRefs.obstacleMeshes ?? [], false);
      if (editorState.mode === "select") {
        if (hits.length > 0) {
          const obstacleId = hits[0].object.userData.obstacleId;
          const obstacle = getObstacleById(obstacleId);
          selectObstacle(obstacleId);
          if (obstacle) {
            const center = {
              x: obstacle.x + obstacle.w / 2,
              y: obstacle.y + obstacle.d / 2,
            };
            const groundHit = intersectGroundPlaneAtY(obstacle.elevation ?? 0);
            if (groundHit) {
              const mapPoint = worldToMapCoords(planeHit.x, planeHit.z);
              editorState.armedObstacleId = obstacleId;
              editorState.armedPointerId = event.pointerId;
              editorState.armedOffsetX = center.x - mapPoint.x;
              editorState.armedOffsetY = center.y - mapPoint.y;
              editorState.armedStartX = event.clientX;
              editorState.armedStartY = event.clientY;
              editorState.armedPlaneY = obstacle.elevation ?? 0;
              renderer.domElement.setPointerCapture(event.pointerId);
            }
          }
        } else {
          selectObstacle(null);
          if (event.button === 0) {
            beginCameraDrag(event, "orbit");
          }
        }
        return;
      }

      if (editorState.mode === "delete") {
        if (hits.length > 0) {
          mapState.obstacles = mapState.obstacles.filter(
            (obstacle) => obstacle.id !== hits[0].object.userData.obstacleId
          );
          if (editorState.selectedObstacleId === hits[0].object.userData.obstacleId) {
            selectObstacle(null);
          }
          refreshSceneGeometry();
          clearPathState();
          drawEditorView();
          updateEditorStatus("Obstacle deleted.");
        }
        return;
      }

      if (!intersectGroundPlaneAtY(0)) {
        return;
      }

      const mapPoint = worldToMapCoords(planeHit.x, planeHit.z);
      if (editorState.mode === "spawn") {
        cfg.go2_pose.x_m = Number(clamp(mapPoint.x, 0, cfg.map.width_m).toFixed(2));
        cfg.go2_pose.y_m = Number(clamp(mapPoint.y, 0, cfg.map.height_m).toFixed(2));
        drawEditorView();
        updateEditorStatus("Spawn point moved.");
        return;
      }

      if (editorState.mode === "add") {
        const w = clamp(Number(document.getElementById("editor-obstacle-width").value) || 0.8, 0.2, 6);
        const d = clamp(Number(document.getElementById("editor-obstacle-depth").value) || 0.8, 0.2, 6);
        const h = clamp(Number(document.getElementById("editor-obstacle-height").value) || 0.6, 0.1, 4);
        const obstacle = {
          id: mapState.nextObstacleId++,
          x: clamp(mapPoint.x - w / 2, 0, Math.max(0, cfg.map.width_m - w)),
          y: clamp(mapPoint.y - d / 2, 0, Math.max(0, cfg.map.height_m - d)),
          w,
          d,
          h,
          elevation: 0,
        };
        mapState.obstacles.push(obstacle);
        refreshSceneGeometry();
        selectObstacle(obstacle.id);
        clearPathState();
        drawEditorView();
        updateEditorStatus("Obstacle added.");
        return;
      }
    }

    if (plannerState.mode) {
      if (event.button !== 0) {
        return;
      }
      if (raycaster.ray.intersectPlane(floorPlane, planeHit)) {
        const clampedPoint = {
          x: clamp(planeHit.x, -halfWidth, halfWidth),
          z: clamp(planeHit.z, -halfHeight, halfHeight),
        };
        const inflatedBounds = inflateBounds(obstacleBounds, robotCollisionRadius + 0.03);
        if (inflatedBounds.some((rectBox) => pointInRect(clampedPoint, rectBox))) {
          updatePlannerStatus("Selected point is too close to an obstacle.");
          return;
        }

        const supportY = getSupportHeightAt(clampedPoint);
        if (plannerState.mode === "start") {
          plannerState.start = clampedPoint;
          setMarkerPosition(startMarker, clampedPoint, supportY);
          updatePlannerStatus("Start point set.");
        } else {
          plannerState.goal = clampedPoint;
          setMarkerPosition(goalMarker, clampedPoint, supportY);
          updatePlannerStatus("Goal point set.");
        }
        clearPlannedPath();
        setPlannerMode(null);
      }
      return;
    }

    if (event.button === 2) {
      if (cameraState.mode === "free") {
        beginCameraDrag(event, "pan");
      }
      return;
    }
    if (event.button === 0) {
      beginCameraDrag(event, "orbit");
    }
  }

  function onPointerMove(event) {
    if (
      editorState.isOpen &&
      editorState.resizePointerId === event.pointerId &&
      editorState.resizingHandleAxis
    ) {
      const obstacle = getObstacleById(editorState.resizeObstacleId);
      if (!obstacle) {
        return;
      }

      if (editorState.resizingHandleAxis === "x" || editorState.resizingHandleAxis === "z") {
        setPointerFromEvent(event);
        if (intersectGroundPlaneAtY(editorState.resizePlaneY)) {
          const mapPoint = worldToMapCoords(planeHit.x, planeHit.z);
          if (editorState.resizingHandleAxis === "x") {
            const maxX = clamp(mapPoint.x, obstacle.x + 0.2, cfg.map.width_m);
            obstacle.w = clamp(maxX - obstacle.x, 0.2, 6);
          } else {
            const maxY = clamp(mapPoint.y, obstacle.y + 0.2, cfg.map.height_m);
            obstacle.d = clamp(maxY - obstacle.y, 0.2, 6);
          }
        }
      } else if (editorState.resizingHandleAxis === "y") {
        const deltaY = event.clientY - editorState.resizeStartClientY;
        obstacle.h = clamp(editorState.resizeBaseHeight - deltaY * 0.01, 0.1, 4);
      }

      refreshSceneGeometry();
      selectObstacle(obstacle.id);
      clearPathState();
      drawEditorView();
      return;
    }

    if (
      editorState.isOpen &&
      editorState.armedPointerId === event.pointerId &&
      !editorState.draggingObstacleId
    ) {
      const distance = Math.hypot(
        event.clientX - editorState.armedStartX,
        event.clientY - editorState.armedStartY
      );
      if (distance > 6) {
        editorState.draggingObstacleId = editorState.armedObstacleId;
        editorState.dragPointerId = editorState.armedPointerId;
        editorState.dragOffsetX = editorState.armedOffsetX;
        editorState.dragOffsetY = editorState.armedOffsetY;
      }
    }

    if (
      editorState.isOpen &&
      editorState.draggingObstacleId &&
      editorState.dragPointerId === event.pointerId
    ) {
      const obstacle = getObstacleById(editorState.draggingObstacleId);
      if (!obstacle) {
        return;
      }
      setPointerFromEvent(event);
      if (intersectGroundPlaneAtY(obstacle.elevation ?? 0)) {
        const mapPoint = worldToMapCoords(planeHit.x, planeHit.z);
        const centerX = clamp(
          mapPoint.x + editorState.dragOffsetX,
          obstacle.w / 2,
          cfg.map.width_m - obstacle.w / 2
        );
        const centerY = clamp(
          mapPoint.y + editorState.dragOffsetY,
          obstacle.d / 2,
          cfg.map.height_m - obstacle.d / 2
        );
        obstacle.x = centerX - obstacle.w / 2;
        obstacle.y = centerY - obstacle.d / 2;
        refreshSceneGeometry();
        selectObstacle(obstacle.id);
        clearPathState();
        drawEditorView();
      }
      return;
    }

    if (!orbitState.dragging || orbitState.pointerId !== event.pointerId) {
      return;
    }

    const deltaX = event.clientX - orbitState.lastX;
    const deltaY = event.clientY - orbitState.lastY;
    orbitState.lastX = event.clientX;
    orbitState.lastY = event.clientY;

    if ((editorState.isOpen || cameraState.mode === "free") && orbitState.dragMode === "pan") {
      if (modeState.mapMode === "tiles3d") {
        const panScale = Math.max(orbitState.distance * 0.0022, 0.0035);
        orbitState.target.addScaledVector(orbitState.panRight, -deltaX * panScale);
        orbitState.target.addScaledVector(orbitState.panUp, deltaY * panScale);
        clampOrbitTarget();
        return;
      }

      const currentUp = getCurrentUpAxis();
      camera.getWorldDirection(cameraForward);
      cameraForward.addScaledVector(currentUp, -cameraForward.dot(currentUp));
      if (cameraForward.lengthSq() < 1e-6) {
        cameraForward.copy(modeState.mapMode === "tiles3d" && modeState.tilesFrame ? modeState.tilesFrame.north : new THREE.Vector3(0, 0, -1));
      } else {
        cameraForward.normalize();
      }
      cameraRight.crossVectors(cameraForward, currentUp).normalize();
      const panScale = Math.max(orbitState.distance * 0.0022, 0.0035);
      orbitState.target.addScaledVector(cameraRight, -deltaX * panScale);
      orbitState.target.addScaledVector(cameraForward, deltaY * panScale);
      clampOrbitTarget();
      return;
    }

    orbitState.azimuth -= deltaX * 0.01;
    orbitState.polar = modeState.mapMode === "tiles3d"
      ? clamp(orbitState.polar + deltaY * 0.008, 0.08, Math.PI - 0.08)
      : clamp(orbitState.polar + deltaY * 0.008, 0.35, 1.35);
  }

  function onPointerUp(event) {
    if (editorState.resizePointerId === event.pointerId) {
      editorState.resizingHandleAxis = null;
      editorState.resizePointerId = null;
      editorState.resizeObstacleId = null;
      renderer.domElement.releasePointerCapture(event.pointerId);
      updateEditorStatus("Obstacle size updated.");
      return;
    }
    if (editorState.armedPointerId === event.pointerId) {
      editorState.armedObstacleId = null;
      editorState.armedPointerId = null;
      editorState.armedOffsetX = 0;
      editorState.armedOffsetY = 0;
      if (!editorState.dragPointerId) {
        renderer.domElement.releasePointerCapture(event.pointerId);
      }
    }
    if (editorState.dragPointerId === event.pointerId) {
      editorState.draggingObstacleId = null;
      editorState.dragPointerId = null;
      editorState.dragOffsetX = 0;
      editorState.dragOffsetY = 0;
      renderer.domElement.releasePointerCapture(event.pointerId);
      updateEditorStatus("Obstacle moved.");
      return;
    }
    if (orbitState.pointerId === event.pointerId) {
      orbitState.dragging = false;
      renderer.domElement.releasePointerCapture(event.pointerId);
      orbitState.pointerId = null;
      orbitState.dragMode = "orbit";
    }
  }

  function onWheel(event) {
    event.preventDefault();
    if (modeState.mapMode === "tiles3d") {
      const zoomFactor = Math.exp(event.deltaY * 0.0018);
      orbitState.distance = clamp(orbitState.distance * zoomFactor, 0.2, 100000);
      return;
    }

    orbitState.distance = clamp(
      orbitState.distance + event.deltaY * 0.0035,
      editorState.isOpen || cameraState.mode === "free" ? 3.2 : 2.1,
      editorState.isOpen || cameraState.mode === "free" ? 22 : 9.5
    );
  }

  renderer.domElement.addEventListener("pointerdown", onPointerDown);
  renderer.domElement.addEventListener("pointermove", onPointerMove);
  renderer.domElement.addEventListener("pointerup", onPointerUp);
  renderer.domElement.addEventListener("pointercancel", onPointerUp);
  renderer.domElement.addEventListener("dblclick", focusOrbitTargetFromPointer);
  renderer.domElement.addEventListener("wheel", onWheel, { passive: false });
  renderer.domElement.addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });
  hydrateHudButtons();
  hydrateGuidePanel();
  if (appMode === "viewer") {
    modeState.mapMode = "sandbox";
    loadModeMotionState("sandbox");
    resetRobotsToConfigPose();
    resetSandboxCamera();
    camera.up.copy(upAxis);
    syncMapModeSceneVisibility();
  } else if (appMode === "tiles") {
    modeState.mapMode = "tiles3d";
    loadModeMotionState("tiles3d");
    loadViewState("tiles3d");
    camera.up.copy(upAxis);
    syncMapModeSceneVisibility();
    startMarker.visible = false;
    goalMarker.visible = false;
    routeGroup.visible = false;
    for (const robot of Object.values(robots)) {
      robot.root.visible = false;
    }
    updatePlannerStatus("Load a 3D tileset to begin.");
  }
  syncCameraModeButton();
  syncGuideVisibility();
  syncTilesetControls();
  syncPositionPanel();
  if (appMode === "editor") {
    showEditorScreen(true);
  }
  document.getElementById("camera-mode-btn")?.addEventListener("click", () => {
    setCameraMode(cameraState.mode === "follow" ? "free" : "follow");
  });
  document.getElementById("guide-toggle-btn")?.addEventListener("click", () => {
    toggleGuideVisibility();
  });
  document.getElementById("position-toggle-btn")?.addEventListener("click", () => {
    modeState.positionPanelOpen = !modeState.positionPanelOpen;
    syncTilesetControls();
    syncPositionPanel();
  });
  document.getElementById("position-float-btn")?.addEventListener("click", () => {
    modeState.positionPanelOpen = !modeState.positionPanelOpen;
    syncTilesetControls();
    syncPositionPanel();
  });
  document.getElementById("set-start-btn").addEventListener("click", () => {
    setStartToRobotPosition();
  });
  document.getElementById("set-goal-btn").addEventListener("click", () => {
    const nextMode = plannerState.mode === "goal" ? null : "goal";
    setPlannerMode(nextMode);
    updatePlannerStatus(nextMode ? "Click the map to place the goal point." : "Select start and goal points.");
  });
  document.getElementById("plan-path-btn").addEventListener("click", () => {
    planCurrentPath();
  });
  document.getElementById("auto-nav-btn").addEventListener("click", () => {
    if (plannerState.autoActive) {
      stopAutoNav("Auto navigation stopped.");
    } else {
      beginAutoNav();
    }
  });
  document.getElementById("robot-select").addEventListener("change", (event) => {
    switchRobot(event.target.value);
  });
  document.getElementById("map-edit-btn").addEventListener("click", () => {
    saveStoredMapState(cfg, mapState);
    window.location.href = "./editor.html";
  });
  document.getElementById("tiles-page-btn")?.addEventListener("click", () => {
    saveStoredMapState(cfg, mapState);
    window.location.href = "./tiles.html";
  });
  document.getElementById("viewer-page-btn")?.addEventListener("click", () => {
    saveStoredMapState(cfg, mapState);
    window.location.href = "./index.html";
  });
  document.getElementById("tiles-mode-btn")?.addEventListener("click", async () => {
    modeState.tilesetPanelOpen = !modeState.tilesetPanelOpen;
    syncTilesetControls();
  });
  document.getElementById("tileset-load-btn")?.addEventListener("click", async () => {
    const nextUrl = document.getElementById("tileset-url-input")?.value ?? modeState.tilesetUrl;
    modeState.tilesetUrl = nextUrl.trim() || modeState.tilesetUrl;
    await setMapMode("tiles3d", { forceReload: true });
  });
  document.getElementById("tileset-preset-select")?.addEventListener("change", (event) => {
    const nextUrl = event.target.value;
    if (!nextUrl) {
      return;
    }
    modeState.tilesetUrl = nextUrl;
    saveStoredTilesetUrl(modeState.tilesetUrl);
    const input = document.getElementById("tileset-url-input");
    if (input) {
      input.value = nextUrl;
    }
    syncTilesetControls();
  });
  document.getElementById("tiles-spawn-btn")?.addEventListener("click", () => {
    if (modeState.mapMode !== "tiles3d" || !modeState.tilesetLoaded) {
      updatePlannerStatus("Load a 3D Tiles tileset before spawning the robot.");
      return;
    }
    modeState.tilesSpawnArmed = true;
    syncTilesetControls();
    updatePlannerStatus(
      modeState.tilesSpawned
        ? "Respawn armed. Click a 3D Tiles surface to move the robot."
        : "Spawn armed. Click a 3D Tiles surface to place the robot."
    );
  });
  document.getElementById("tileset-sandbox-btn")?.addEventListener("click", async () => {
    await setMapMode("sandbox");
  });
  document.getElementById("tileset-url-input")?.addEventListener("change", (event) => {
    modeState.tilesetUrl = event.target.value.trim() || modeState.tilesetUrl;
    saveStoredTilesetUrl(modeState.tilesetUrl);
    syncTilesetControls();
  });
  if (appMode === "editor") {
    document.getElementById("editor-close-btn").addEventListener("click", () => {
      saveStoredMapState(cfg, mapState);
      window.location.href = "./index.html";
    });
    document.getElementById("editor-select-btn").addEventListener("click", () => {
      setEditorMode("select");
      updateEditorStatus("Click an obstacle to select and drag it.");
    });
    document.getElementById("editor-add-btn").addEventListener("click", () => {
      setEditorMode("add");
      updateEditorStatus("Click the 3D map to place an obstacle.");
    });
    document.getElementById("editor-delete-btn").addEventListener("click", () => {
      setEditorMode("delete");
      updateEditorStatus("Click an obstacle in 3D to delete it.");
    });
    document.getElementById("editor-spawn-btn").addEventListener("click", () => {
      setEditorMode("spawn");
      updateEditorStatus("Click the 3D map to place the spawn point.");
    });
    document.getElementById("editor-apply-btn").addEventListener("click", () => {
      applyMapChangesFromEditor();
    });
    document.getElementById("editor-canvas").addEventListener("click", (event) => {
      const point = editorCanvasToMapPoint(event);
      if (editorState.mode === "add") {
        const w = clamp(Number(document.getElementById("editor-obstacle-width").value) || 0.8, 0.2, 6);
        const d = clamp(Number(document.getElementById("editor-obstacle-depth").value) || 0.8, 0.2, 6);
        const h = clamp(Number(document.getElementById("editor-obstacle-height").value) || 0.6, 0.1, 2);
        mapState.obstacles.push({
          x: clamp(point.x - w / 2, 0, Math.max(0, cfg.map.width_m - w)),
          y: clamp(point.y - d / 2, 0, Math.max(0, cfg.map.height_m - d)),
          w,
          d,
          h,
        });
        drawEditorView();
        updateEditorStatus("Obstacle added.");
      } else if (editorState.mode === "delete") {
        const index = mapState.obstacles.findIndex((obstacle) =>
          point.x >= obstacle.x &&
          point.x <= obstacle.x + obstacle.w &&
          point.y >= obstacle.y &&
          point.y <= obstacle.y + obstacle.d
        );
        if (index >= 0) {
          mapState.obstacles.splice(index, 1);
          drawEditorView();
          updateEditorStatus("Obstacle deleted.");
        }
      } else if (editorState.mode === "spawn") {
        cfg.go2_pose.x_m = Number(point.x.toFixed(2));
        cfg.go2_pose.y_m = Number(point.y.toFixed(2));
        drawEditorView();
        updateEditorStatus("Spawn point moved.");
      }
    });
    [
      "editor-box-x",
      "editor-box-y",
      "editor-box-elevation",
      "editor-box-height",
      "editor-box-width",
      "editor-box-depth",
    ].forEach((id) => {
      document.getElementById(id).addEventListener("input", () => {
        const obstacle = getObstacleById(editorState.selectedObstacleId);
        if (!obstacle) {
          return;
        }
        const nextX = Number(document.getElementById("editor-box-x").value);
        const nextY = Number(document.getElementById("editor-box-y").value);
        const nextElevation = Number(document.getElementById("editor-box-elevation").value);
        const nextHeight = Number(document.getElementById("editor-box-height").value);
        const nextWidth = Number(document.getElementById("editor-box-width").value);
        const nextDepth = Number(document.getElementById("editor-box-depth").value);
        obstacle.w = clamp(Number.isFinite(nextWidth) ? nextWidth : obstacle.w, 0.2, 6);
        obstacle.d = clamp(Number.isFinite(nextDepth) ? nextDepth : obstacle.d, 0.2, 6);
        obstacle.h = clamp(Number.isFinite(nextHeight) ? nextHeight : obstacle.h, 0.1, 4);
        obstacle.elevation = clamp(Number.isFinite(nextElevation) ? nextElevation : obstacle.elevation ?? 0, 0, 4);
        obstacle.x = clamp(Number.isFinite(nextX) ? nextX : obstacle.x, 0, Math.max(0, cfg.map.width_m - obstacle.w));
        obstacle.y = clamp(Number.isFinite(nextY) ? nextY : obstacle.y, 0, Math.max(0, cfg.map.height_m - obstacle.d));
        refreshSceneGeometry();
        selectObstacle(obstacle.id);
        clearPathState();
        drawEditorView();
        updateEditorStatus("Selected obstacle updated.");
      });
    });
  }

  function resize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
    if (sceneRefs.tilesRenderer) {
      sceneRefs.tilesRenderer.setResolutionFromRenderer(camera, renderer);
    }
  }

  const clock = new THREE.Clock();

  updateGo2Pose(go2Rig, motionState, 0, {
    speed: 0,
    cadence: 3.2,
    forward: 0,
    lateral: 0,
    turn: 0,
    forwardAmount: 0,
    lateralAmount: 0,
    turnAmount: 0,
    sneak: false,
  });
  robots.go2.groundOffset = calibrateGroundOffset(go2Rig);
  robots.turtlebot3.groundOffset = turtlebot3Spec.wheelRadius - turtlebotWheelCenterHeight;
  motionState.groundOffset = activeRobot.groundOffset;
  turtlebot3Rig.root.visible = false;
  if (appMode === "tiles") {
    plannerState.start = null;
    startMarker.visible = false;
    goalMarker.visible = false;
  } else {
    plannerState.start = worldToMapPoint(activeRobot.root.position);
    setMarkerPosition(startMarker, plannerState.start, motionState.supportY);
  }

  function animate() {
    const delta = Math.min(clock.getDelta(), 0.05);
    const elapsed = clock.elapsedTime;
    const pulse = 1 + Math.sin(elapsed * 1.8) * 0.035;

    let turnInput = (inputState.turnLeft ? 1 : 0) - (inputState.turnRight ? 1 : 0);
    let forwardInput = (inputState.forward ? 1 : 0) - (inputState.backward ? 1 : 0);
    let lateralInput = (inputState.right ? 1 : 0) - (inputState.left ? 1 : 0);
    let currentMoveSpeed = walkSpeed;
    let autoMoveDirection = null;
    let directAutoMotion = false;
    let directAutoSpeed = 0;
    let poseForwardInput = forwardInput;
    let poseLateralInput = lateralInput;
    let poseTurnInput = turnInput;
    if (inputState.sneak) {
      currentMoveSpeed = sneakSpeed;
    } else if (inputState.sprint && inputState.forward) {
      currentMoveSpeed = sprintSpeed;
    }

    if (plannerState.autoActive) {
      const followerModule = plannerState.followerKey ? followerModules[plannerState.followerKey] : null;
      if (followerModule && plannerState.followerState) {
        const followerResult = followerModule.step({
          plannerState,
          motionState,
          activeRobot,
          obstacleBounds,
          delta,
          turnSpeed,
          autoNavSpeed,
          sprintSpeed,
          halfWidth: modeState.mapMode === "tiles3d" ? Number.POSITIVE_INFINITY : halfWidth,
          halfHeight: modeState.mapMode === "tiles3d" ? Number.POSITIVE_INFINITY : halfHeight,
          autoCommandState,
          resolveMotionWithSteps: modeState.mapMode === "tiles3d"
            ? (nextX, nextZ, currentX, currentZ, currentSupportY, radius) =>
              resolveTilesMotion(nextX, nextZ, currentX, currentZ, currentSupportY, radius)
            : resolveMotionWithSteps,
          projectPointOntoPath,
          samplePathAtProgress,
        });

        if (followerResult?.done) {
          plannerState.start = modeState.mapMode === "tiles3d"
            ? { x: motionState.x, z: motionState.z, y: motionState.supportY }
            : worldToMapPoint(activeRobot.root.position);
          setMarkerPosition(startMarker, plannerState.start, motionState.supportY);
          stopAutoNav(followerResult.statusText ?? "Goal reached.");
        } else if (followerResult?.command) {
          directAutoMotion = followerResult.command.directAutoMotion ?? false;
          directAutoSpeed = followerResult.command.directAutoSpeed ?? directAutoSpeed;
          forwardInput = followerResult.command.forwardInput ?? forwardInput;
          lateralInput = followerResult.command.lateralInput ?? lateralInput;
          turnInput = followerResult.command.turnInput ?? turnInput;
          poseForwardInput = followerResult.command.poseForwardInput ?? forwardInput;
          poseLateralInput = followerResult.command.poseLateralInput ?? lateralInput;
          poseTurnInput = followerResult.command.poseTurnInput ?? turnInput;
          autoMoveDirection = followerResult.command.autoMoveDirection ?? autoMoveDirection;
          if (typeof followerResult.command.currentMoveSpeed === "number") {
            currentMoveSpeed = followerResult.command.currentMoveSpeed;
          }
        }
      }
    }

    if (activeRobot.type === "turtlebot3") {
      lateralInput = 0;
    }

    if (modeState.mapMode === "tiles3d" && !modeState.tilesSpawned) {
      turnInput = 0;
      forwardInput = 0;
      lateralInput = 0;
      poseForwardInput = 0;
      poseLateralInput = 0;
      poseTurnInput = 0;
    }

    if (modeState.mapMode === "tiles3d" && !plannerState.autoActive) {
      // In the anchored 3D Tiles frame, manual key yaw is mirrored relative to
      // the sandbox sign convention. Keep autonomous follower yaw untouched.
      turnInput *= -1;
      poseTurnInput *= -1;
    }

    if (!directAutoMotion) {
      motionState.yaw += turnInput * turnSpeed * delta;

      if (autoMoveDirection) {
        moveDirection.set(autoMoveDirection.x, 0, autoMoveDirection.z);
      } else if (modeState.mapMode === "tiles3d" && modeState.tilesFrame && modeState.tilesSpawned) {
        moveDirection.set(
          Math.cos(motionState.yaw) * forwardInput - Math.sin(motionState.yaw) * lateralInput,
          0,
          -Math.sin(motionState.yaw) * forwardInput - Math.cos(motionState.yaw) * lateralInput
        );
      } else if (modeState.mapMode === "tiles3d") {
        moveDirection.set(0, 0, 0);
      } else {
        moveDirection.set(
          Math.cos(motionState.yaw) * forwardInput + Math.sin(motionState.yaw) * lateralInput,
          0,
          -Math.sin(motionState.yaw) * forwardInput + Math.cos(motionState.yaw) * lateralInput
        );
      }
    } else {
      moveDirection.set(0, 0, 0);
    }

    let normalizedSpeed = 0;

    if (directAutoMotion) {
      normalizedSpeed = directAutoSpeed;
    } else if (moveDirection.lengthSq() > 0) {
      moveDirection.normalize();
      normalizedSpeed = currentMoveSpeed / sprintSpeed;
      const rawTargetX = motionState.x + moveDirection.x * currentMoveSpeed * delta;
      const rawTargetZ = motionState.z + moveDirection.z * currentMoveSpeed * delta;
      const targetX = modeState.mapMode === "tiles3d" ? rawTargetX : clamp(rawTargetX, -halfWidth, halfWidth);
      const targetZ = modeState.mapMode === "tiles3d" ? rawTargetZ : clamp(rawTargetZ, -halfHeight, halfHeight);
      const effectiveStepHeight =
        activeRobot.stepHeight +
        (motionState.isGrounded ? 0 : jumpStepBonus) +
        clamp(motionState.jumpY * 0.45, 0, 0.16);

      const resolved = modeState.mapMode === "tiles3d"
        ? resolveTilesMotion(
          targetX,
          targetZ,
          motionState.x,
          motionState.z,
          motionState.supportY,
          activeRobot.collisionRadius
        )
        : resolveMotionWithSteps(
          targetX,
          targetZ,
          motionState.x,
          motionState.z,
          motionState.supportY,
          activeRobot.collisionRadius,
          effectiveStepHeight,
          obstacleBounds
        );
      motionState.x = resolved.x;
      motionState.z = resolved.z;
      if (motionState.isGrounded) {
        motionState.supportY = resolved.supportY;
      }
    } else if (motionState.isGrounded) {
      motionState.supportY = modeState.mapMode === "tiles3d"
        ? (findTilesSupportY(motionState.x, motionState.z, motionState.supportY) ?? motionState.supportY)
        : evaluateSupportSurface(
          motionState.x,
          motionState.z,
          activeRobot.collisionRadius,
          motionState.supportY,
          activeRobot.stepHeight,
          obstacleBounds
        ).supportY;
    }

    if (!motionState.isGrounded || motionState.jumpY > 0) {
      motionState.velocityY -= gravity * delta;
      motionState.jumpY += motionState.velocityY * delta;

      if (motionState.jumpY <= 0) {
        motionState.supportY = modeState.mapMode === "tiles3d"
          ? (findTilesSupportY(motionState.x, motionState.z, motionState.supportY) ?? motionState.supportY)
          : evaluateSupportSurface(
            motionState.x,
            motionState.z,
            activeRobot.collisionRadius,
            Math.max(motionState.supportY, activeRobot.stepHeight),
            Number.POSITIVE_INFINITY,
            obstacleBounds
          ).supportY;
        motionState.jumpY = 0;
        motionState.velocityY = 0;
        motionState.isGrounded = true;
      }
    }

    const crouchOffset = inputState.sneak ? -0.08 : 0;
    const idleBob = 0;
    motionState.groundOffset = activeRobot.groundOffset + idleBob + crouchOffset;
    syncActiveRobotTransform();
    motionState.groundOffset = activeRobot.groundOffset;

    if (activeRobot.type === "go2") {
      updateGo2Pose(go2Rig, motionState, elapsed, {
        speed: normalizedSpeed,
        cadence: THREE.MathUtils.lerp(
          3.4,
          9.4,
          Math.max(normalizedSpeed, Math.abs(poseTurnInput) * 0.9)
        ),
        forward: poseForwardInput,
        lateral: poseLateralInput,
        turn: poseTurnInput,
        forwardAmount: Math.abs(poseForwardInput) * Math.max(normalizedSpeed, 0.6),
        lateralAmount: Math.abs(poseLateralInput) * Math.max(normalizedSpeed, 0.45),
        turnAmount: Math.abs(poseTurnInput) * Math.max(normalizedSpeed, 0.62),
        sneak: inputState.sneak,
      });
    } else {
      const linearVelocity = currentMoveSpeed * forwardInput;
      const yawRate = turnInput * turnSpeed;
      const leftWheelLinear = linearVelocity - yawRate * (turtlebot3Spec.wheelSeparation * 0.5);
      const rightWheelLinear = linearVelocity + yawRate * (turtlebot3Spec.wheelSeparation * 0.5);
      turtlebot3Rig.wheelLeftSpin.rotation.y -= (leftWheelLinear / turtlebot3Spec.wheelRadius) * delta;
      turtlebot3Rig.wheelRightSpin.rotation.y -= (rightWheelLinear / turtlebot3Spec.wheelRadius) * delta;
    }

    scanRing.scale.setScalar(pulse);
    scanRing.position.copy(activeRobot.root.position);
    if (modeState.mapMode === "tiles3d" && modeState.tilesFrame) {
      const ringQuat = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), modeState.tilesFrame.up);
      scanRing.quaternion.copy(ringQuat);
      scanRing.position.addScaledVector(modeState.tilesFrame.up, -motionState.groundOffset * 0.92);
    } else {
      scanRing.rotation.set(0, 0, 0);
      scanRing.position.y = 0;
    }

    const currentUp = getCurrentUpAxis();
    camera.up.copy(currentUp);
    const horizontalRadius = Math.cos(orbitState.polar) * orbitState.distance;
    const verticalOffset = Math.sin(orbitState.polar) * orbitState.distance;
    chaseOffset.set(-horizontalRadius, verticalOffset, 0);
    if (editorState.isOpen || cameraState.mode === "free") {
      chaseOffset.applyAxisAngle(currentUp, orbitState.azimuth);
      chasePosition.copy(orbitState.target).add(chaseOffset);
      camera.position.lerp(chasePosition, 0.18);
      chaseLookAt.copy(orbitState.target);
    } else {
      const cameraYaw = motionState.yaw + orbitState.azimuth;
      chaseOffset.applyAxisAngle(currentUp, cameraYaw);
      chasePosition.copy(activeRobot.root.position).add(chaseOffset);
      camera.position.lerp(chasePosition, 0.12);
      const lookForward = new THREE.Vector3();
      if (modeState.mapMode === "tiles3d" && modeState.tilesFrame) {
        lookForward
          .copy(modeState.tilesFrame.east)
          .multiplyScalar(Math.cos(motionState.yaw))
          .addScaledVector(modeState.tilesFrame.north, -Math.sin(motionState.yaw))
          .normalize();
      } else {
        lookForward.set(Math.cos(motionState.yaw), 0, -Math.sin(motionState.yaw));
      }
      chaseLookAt.copy(activeRobot.root.position);
      chaseLookAt.addScaledVector(lookForward, modeState.mapMode === "tiles3d" ? 0.15 : 0.55);
      chaseLookAt.addScaledVector(currentUp, modeState.mapMode === "tiles3d" ? 0.18 : 0.42 + motionState.jumpY * 0.25 + crouchOffset * 0.3);
    }
    camera.lookAt(chaseLookAt);
    updateOrbitReticle();

    syncPoseToHud();
    syncPositionPanel();

    if (modeState.mapMode === "tiles3d" && sceneRefs.tilesRenderer) {
      camera.updateMatrixWorld();
      if (tilesRefreshFrames > 0) {
        sceneRefs.tilesRenderer.setResolutionFromRenderer(camera, renderer);
        tilesRefreshFrames -= 1;
      }
      sceneRefs.tilesRenderer.update();
    }

    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  window.addEventListener("resize", resize);
  animate();
}
