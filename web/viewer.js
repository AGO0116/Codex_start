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
import {
  applyStoredMapState,
  createRuntimeMapState,
  loadStoredMapState,
  saveStoredMapState,
} from "./modules/map-state.js";
import { drawPlannerDebugView, showPlannerDebugPanel } from "./modules/planner-debug-view.js";

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
  scene.fog = new THREE.Fog(0x05070b, 18, 54);

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

async function main(appModeOverride) {
  const cfg = await loadConfig();
  const appMode = appModeOverride ?? document.body.dataset.appMode ?? "viewer";
  const storedMapState = loadStoredMapState();
  applyStoredMapState(cfg, storedMapState);
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
  const { scene, scanRing } = sceneRefs;
  const go2Rig = createGo2Model(go2Assets);
  const turtlebot3Rig = createTurtlebot3Model(turtlebot3Assets);
  placeGo2(go2Rig.root, cfg);
  placeGo2(turtlebot3Rig.root, cfg);
  scene.add(go2Rig.root);
  scene.add(turtlebot3Rig.root);
  updateHud(cfg);

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
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const planeHit = new THREE.Vector3();
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
  };
  const cameraState = {
    mode: appMode === "editor" ? "free" : "follow",
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

  function syncActiveRobotTransform() {
    for (const robot of Object.values(robots)) {
      robot.root.visible = robot === activeRobot;
      robot.root.position.set(motionState.x, motionState.groundOffset + motionState.supportY + motionState.jumpY, motionState.z);
      robot.root.rotation.y = motionState.yaw;
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
    marker.position.set(point.x, supportY + 0.07, point.z);
    marker.visible = true;
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
    showPlannerDebugPanel(false);
    stopAutoNav();
  }

  function updateEditorStatus(text) {
    const node = document.querySelector('[data-role="editor-status"]');
    if (node) {
      node.textContent = text;
    }
  }

  function clampOrbitTarget() {
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
      orbitState.target.set(motionState.x, 0.32, motionState.z);
      clampOrbitTarget();
    }
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
      "map-edit-btn": ["Map Edit", "Open 3D editor"],
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
    document.getElementById("map-edit-btn")?.classList.add("accent-button");
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
      [["LMB"], "Orbit camera in free view"],
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
      [["LMB", "Empty"], "Orbit on empty space"],
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
    const startPoint = worldToMapPoint(activeRobot.root.position);
    plannerState.start = startPoint;
    setMarkerPosition(startMarker, startPoint, motionState.supportY);
    clearPathState();
    setPlannerMode("start");
    updatePlannerStatus("Start point set to robot position. Click the map to move it.");
  }

  function planCurrentPath() {
    if (!plannerState.start || !plannerState.goal) {
      updatePlannerStatus("Set both start and goal first.");
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
    cfg.go2_pose.x_m = Number((motionState.x + cfg.map.width_m / 2).toFixed(2));
    cfg.go2_pose.y_m = Number((motionState.z + cfg.map.height_m / 2).toFixed(2));
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
    }
    renderer.domElement.setPointerCapture(event.pointerId);
  }

  function onPointerDown(event) {
    setPointerFromEvent(event);

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
      camera.getWorldDirection(cameraForward);
      cameraForward.y = 0;
      if (cameraForward.lengthSq() < 1e-6) {
        cameraForward.set(0, 0, -1);
      } else {
        cameraForward.normalize();
      }
      cameraRight.crossVectors(cameraForward, upAxis).normalize();
      const panScale = Math.max(orbitState.distance * 0.0022, 0.0035);
      orbitState.target.addScaledVector(cameraRight, -deltaX * panScale);
      orbitState.target.addScaledVector(cameraForward, deltaY * panScale);
      clampOrbitTarget();
      return;
    }

    orbitState.azimuth -= deltaX * 0.01;
    orbitState.polar = clamp(orbitState.polar + deltaY * 0.008, 0.35, 1.35);
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
  renderer.domElement.addEventListener("wheel", onWheel, { passive: false });
  renderer.domElement.addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });
  hydrateHudButtons();
  hydrateGuidePanel();
  syncCameraModeButton();
  syncGuideVisibility();
  if (appMode === "editor") {
    showEditorScreen(true);
  }
  document.getElementById("camera-mode-btn")?.addEventListener("click", () => {
    setCameraMode(cameraState.mode === "follow" ? "free" : "follow");
  });
  document.getElementById("guide-toggle-btn")?.addEventListener("click", () => {
    toggleGuideVisibility();
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

  function resize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
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
  plannerState.start = worldToMapPoint(activeRobot.root.position);
  setMarkerPosition(startMarker, plannerState.start, motionState.supportY);

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
          halfWidth,
          halfHeight,
          autoCommandState,
          resolveMotionWithSteps,
          projectPointOntoPath,
          samplePathAtProgress,
        });

        if (followerResult?.done) {
          plannerState.start = worldToMapPoint(activeRobot.root.position);
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

    if (!directAutoMotion) {
      motionState.yaw += turnInput * turnSpeed * delta;

      if (autoMoveDirection) {
        moveDirection.set(autoMoveDirection.x, 0, autoMoveDirection.z);
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
      const targetX = clamp(motionState.x + moveDirection.x * currentMoveSpeed * delta, -halfWidth, halfWidth);
      const targetZ = clamp(motionState.z + moveDirection.z * currentMoveSpeed * delta, -halfHeight, halfHeight);
      const effectiveStepHeight =
        activeRobot.stepHeight +
        (motionState.isGrounded ? 0 : jumpStepBonus) +
        clamp(motionState.jumpY * 0.45, 0, 0.16);

      const resolved = resolveMotionWithSteps(
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
      motionState.supportY = evaluateSupportSurface(
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
        motionState.supportY = evaluateSupportSurface(
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
    activeRobot.root.position.set(
      motionState.x,
      motionState.groundOffset + motionState.supportY + motionState.jumpY + idleBob + crouchOffset,
      motionState.z
    );
    activeRobot.root.rotation.y = motionState.yaw;

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
    scanRing.position.set(activeRobot.root.position.x, 0, activeRobot.root.position.z);

    const horizontalRadius = Math.cos(orbitState.polar) * orbitState.distance;
    const verticalOffset = Math.sin(orbitState.polar) * orbitState.distance;
    chaseOffset.set(-horizontalRadius, verticalOffset, 0);
    if (editorState.isOpen || cameraState.mode === "free") {
      chaseOffset.applyAxisAngle(upAxis, orbitState.azimuth);
      chasePosition.copy(orbitState.target).add(chaseOffset);
      camera.position.lerp(chasePosition, 0.18);
      chaseLookAt.copy(orbitState.target);
    } else {
      const cameraYaw = motionState.yaw + orbitState.azimuth;
      chaseOffset.applyAxisAngle(upAxis, cameraYaw);
      chasePosition.copy(activeRobot.root.position).add(chaseOffset);
      camera.position.lerp(chasePosition, 0.12);
      chaseLookAt.set(
        activeRobot.root.position.x + Math.cos(motionState.yaw) * 0.55,
        0.42 + motionState.jumpY * 0.25 + crouchOffset * 0.3,
        activeRobot.root.position.z - Math.sin(motionState.yaw) * 0.55
      );
    }
    camera.lookAt(chaseLookAt);

    syncPoseToHud();

    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  window.addEventListener("resize", resize);
  animate();
}

main();
