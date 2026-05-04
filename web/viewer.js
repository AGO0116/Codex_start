import * as THREE from "three";
import { ColladaLoader } from "three/addons/loaders/ColladaLoader.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const defaultConfig = {
  map: { width_m: 10.0, height_m: 8.0 },
  go2_pose: { x_m: 3.5, y_m: 2.0, yaw_deg: 45.0 },
};

const obstacles = [
  { x: 2.0, y: 1.0, w: 3.0, d: 0.3, h: 0.45 },
  { x: 6.0, y: 3.0, w: 0.4, d: 2.5, h: 0.8 },
  { x: 1.0, y: 5.5, w: 2.8, d: 0.4, h: 0.55 },
  { x: 7.4, y: 0.8, w: 0.55, d: 0.6, h: 0.12 },
  { x: 7.95, y: 0.8, w: 0.55, d: 0.6, h: 0.24 },
  { x: 8.5, y: 0.8, w: 0.55, d: 0.6, h: 0.36 },
  { x: 9.05, y: 0.8, w: 0.55, d: 0.6, h: 0.48 },
];

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

function moveTowards(current, target, maxDelta) {
  if (Math.abs(target - current) <= maxDelta) {
    return target;
  }
  return current + Math.sign(target - current) * maxDelta;
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
}

function createObstacles(scene, cfg) {
  const obstacleGroup = new THREE.Group();
  const widthM = cfg.map.width_m;
  const heightM = cfg.map.height_m;
  const bounds = [];

  const material = new THREE.MeshStandardMaterial({
    color: 0x607089,
    roughness: 0.72,
    metalness: 0.18,
  });

  for (const obstacle of obstacles) {
    const footprint = mapToWorld(
      obstacle.x + obstacle.w / 2,
      obstacle.y + obstacle.d / 2,
      widthM,
      heightM
    );

    const mesh = new THREE.Mesh(
      new THREE.BoxGeometry(obstacle.w, obstacle.h, obstacle.d),
      material
    );
    mesh.position.set(footprint.x, obstacle.h / 2, footprint.z);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    obstacleGroup.add(mesh);
    bounds.push({
      minX: footprint.x - obstacle.w / 2,
      maxX: footprint.x + obstacle.w / 2,
      minZ: footprint.z - obstacle.d / 2,
      maxZ: footprint.z + obstacle.d / 2,
      topY: obstacle.h,
    });
  }

  scene.add(obstacleGroup);
  return bounds;
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

  const wheelLeftPivot = new THREE.Group();
  wheelLeftPivot.position.copy(urdfVectorToThree(0, 0.08, 0.023));
  applyUrdfQuaternion(wheelLeftPivot, -1.57, 0, 0);
  baseFrame.add(wheelLeftPivot);

  const wheelRightPivot = new THREE.Group();
  wheelRightPivot.position.copy(urdfVectorToThree(0, -0.08, 0.023));
  applyUrdfQuaternion(wheelRightPivot, -1.57, 0, 0);
  baseFrame.add(wheelRightPivot);

  if (assets?.leftWheel) {
    const leftWheel = createTurtlebotMesh(assets.leftWheel, wheelMaterial);
    wheelLeftPivot.add(leftWheel);
  }

  if (assets?.rightWheel) {
    const rightWheel = createTurtlebotMesh(assets.rightWheel, wheelMaterial);
    wheelRightPivot.add(rightWheel);
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
    wheelLeftPivot,
    wheelRightPivot,
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

    const forwardSwing = swing * moveState.forward * 0.95;
    const lateralSwing = swing * moveState.lateral * sideSign * 0.75;
    const turnSwing = swing * moveState.turn * frontSign * sideSign * 0.85;
    const compositeSwing = forwardSwing + lateralSwing + turnSwing;

    const liftGain =
      moveState.forwardAmount * 0.85 +
      moveState.lateralAmount * 0.75 +
      moveState.turnAmount * 0.8;
    const compositeLift = lift * liftGain;

    const turnHipBias = moveState.turn * frontSign * sideSign * 0.16;
    const lateralHipBias = moveState.lateral * sideSign * 0.12;

    const hip = clamp(baseHip * sideSign + lateralLean * sideSign - moveState.turn * 0.05 * sideSign, go2Spec.joints.hip.min, go2Spec.joints.hip.max);
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
    const finalHip = clamp(
      hip + lateralHipBias + turnHipBias,
      go2Spec.joints.hip.min,
      go2Spec.joints.hip.max
    );

    applyJointPose(leg, { hip: finalHip, thigh, calf });
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

function createScene(cfg) {
  const scene = new THREE.Scene();
  scene.fog = new THREE.Fog(0x05070b, 7, 20);

  createMapBase(scene, cfg);
  const obstacleBounds = createObstacles(scene, cfg);
  createLights(scene, cfg);

  const scanRing = createScanRing();
  scene.add(scanRing);

  return { scene, scanRing, obstacleBounds };
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
    renderNode.textContent = "Official DAE + URDF Rig";
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

function inflateBounds(bounds, margin) {
  return bounds.map((box) => ({
    minX: box.minX - margin,
    maxX: box.maxX + margin,
    minZ: box.minZ - margin,
    maxZ: box.maxZ + margin,
  }));
}

function pointInRect(point, rect) {
  return (
    point.x > rect.minX &&
    point.x < rect.maxX &&
    point.z > rect.minZ &&
    point.z < rect.maxZ
  );
}

function orientation(a, b, c) {
  return (b.z - a.z) * (c.x - b.x) - (b.x - a.x) * (c.z - b.z);
}

function onSegment(a, b, c) {
  return (
    Math.min(a.x, c.x) <= b.x + 1e-6 &&
    b.x <= Math.max(a.x, c.x) + 1e-6 &&
    Math.min(a.z, c.z) <= b.z + 1e-6 &&
    b.z <= Math.max(a.z, c.z) + 1e-6
  );
}

function segmentsIntersect(p1, q1, p2, q2) {
  const o1 = orientation(p1, q1, p2);
  const o2 = orientation(p1, q1, q2);
  const o3 = orientation(p2, q2, p1);
  const o4 = orientation(p2, q2, q1);

  if ((o1 > 0 && o2 < 0 || o1 < 0 && o2 > 0) && (o3 > 0 && o4 < 0 || o3 < 0 && o4 > 0)) {
    return true;
  }

  if (Math.abs(o1) < 1e-6 && onSegment(p1, p2, q1)) return true;
  if (Math.abs(o2) < 1e-6 && onSegment(p1, q2, q1)) return true;
  if (Math.abs(o3) < 1e-6 && onSegment(p2, p1, q2)) return true;
  if (Math.abs(o4) < 1e-6 && onSegment(p2, q1, q2)) return true;

  return false;
}

function segmentCrossesRect(a, b, rect) {
  if (pointInRect(a, rect) || pointInRect(b, rect)) {
    return true;
  }

  const corners = [
    { x: rect.minX, z: rect.minZ },
    { x: rect.maxX, z: rect.minZ },
    { x: rect.maxX, z: rect.maxZ },
    { x: rect.minX, z: rect.maxZ },
  ];

  for (let index = 0; index < corners.length; index += 1) {
    const c1 = corners[index];
    const c2 = corners[(index + 1) % corners.length];
    if (segmentsIntersect(a, b, c1, c2)) {
      return true;
    }
  }

  return false;
}

function hasLineOfSight(a, b, inflatedBounds) {
  for (const rect of inflatedBounds) {
    if (segmentCrossesRect(a, b, rect)) {
      return false;
    }
  }
  return true;
}

function makeNodeKey(point) {
  return `${point.x.toFixed(4)}:${point.z.toFixed(4)}`;
}

function buildVisibilityGraph(start, goal, inflatedBounds, mapWidth, mapHeight) {
  const margin = 0.04;
  const nodes = [start, goal];

  for (const rect of inflatedBounds) {
    const corners = [
      { x: rect.minX - margin, z: rect.minZ - margin },
      { x: rect.maxX + margin, z: rect.minZ - margin },
      { x: rect.maxX + margin, z: rect.maxZ + margin },
      { x: rect.minX - margin, z: rect.maxZ + margin },
    ];

    for (const corner of corners) {
      if (
        corner.x <= -mapWidth / 2 + margin ||
        corner.x >= mapWidth / 2 - margin ||
        corner.z <= -mapHeight / 2 + margin ||
        corner.z >= mapHeight / 2 - margin
      ) {
        continue;
      }
      if (!inflatedBounds.some((rectTest) => pointInRect(corner, rectTest))) {
        nodes.push(corner);
      }
    }
  }

  const deduped = [];
  const seen = new Set();
  for (const node of nodes) {
    const key = makeNodeKey(node);
    if (!seen.has(key)) {
      seen.add(key);
      deduped.push(node);
    }
  }

  const graph = deduped.map(() => []);
  for (let i = 0; i < deduped.length; i += 1) {
    for (let j = i + 1; j < deduped.length; j += 1) {
      if (!hasLineOfSight(deduped[i], deduped[j], inflatedBounds)) {
        continue;
      }
      const dx = deduped[i].x - deduped[j].x;
      const dz = deduped[i].z - deduped[j].z;
      const distance = Math.hypot(dx, dz);
      graph[i].push({ to: j, cost: distance });
      graph[j].push({ to: i, cost: distance });
    }
  }

  return { nodes: deduped, graph };
}

function dijkstraShortestPath(nodes, graph, startIndex, goalIndex) {
  const dist = nodes.map(() => Number.POSITIVE_INFINITY);
  const prev = nodes.map(() => -1);
  const visited = nodes.map(() => false);
  dist[startIndex] = 0;

  for (let count = 0; count < nodes.length; count += 1) {
    let current = -1;
    let best = Number.POSITIVE_INFINITY;
    for (let i = 0; i < nodes.length; i += 1) {
      if (!visited[i] && dist[i] < best) {
        best = dist[i];
        current = i;
      }
    }

    if (current === -1 || current === goalIndex) {
      break;
    }

    visited[current] = true;
    for (const edge of graph[current]) {
      const alt = dist[current] + edge.cost;
      if (alt < dist[edge.to]) {
        dist[edge.to] = alt;
        prev[edge.to] = current;
      }
    }
  }

  if (!Number.isFinite(dist[goalIndex])) {
    return null;
  }

  const path = [];
  for (let cursor = goalIndex; cursor !== -1; cursor = prev[cursor]) {
    path.push(nodes[cursor]);
  }
  path.reverse();
  return path;
}

function prunePath(path, inflatedBounds) {
  if (!path || path.length <= 2) {
    return path;
  }

  const pruned = [path[0]];
  let anchor = 0;

  while (anchor < path.length - 1) {
    let next = path.length - 1;
    while (next > anchor + 1 && !hasLineOfSight(path[anchor], path[next], inflatedBounds)) {
      next -= 1;
    }
    pruned.push(path[next]);
    anchor = next;
  }

  return pruned;
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

function buildPathRenderables(path) {
  const group = new THREE.Group();

  const polyPoints = path.map((point) => pointToVector(point, 0.06));
  const polyline = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(polyPoints),
    new THREE.LineDashedMaterial({
      color: 0x7ed0ff,
      dashSize: 0.18,
      gapSize: 0.08,
      transparent: true,
      opacity: 0.78,
    })
  );
  polyline.computeLineDistances();
  group.add(polyline);

  let curvePoints = polyPoints;
  if (path.length >= 3) {
    const curve = new THREE.CatmullRomCurve3(polyPoints, false, "centripetal", 0.15);
    curvePoints = curve.getPoints(Math.max(32, path.length * 18));
  }

  const curveLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints(curvePoints.map((point) => point.clone().setY(0.08))),
    new THREE.LineBasicMaterial({
      color: 0xffb176,
      transparent: true,
      opacity: 0.95,
    })
  );
  group.add(curveLine);

  return group;
}

function buildSmoothPath(path) {
  if (!path || path.length < 2) {
    return path;
  }

  if (path.length < 3) {
    return path.map((point) => ({ x: point.x, z: point.z }));
  }

  const curve = new THREE.CatmullRomCurve3(
    path.map((point) => pointToVector(point, 0)),
    false,
    "centripetal",
    0.15
  );

  return curve.getPoints(Math.max(32, path.length * 18)).map((point) => ({
    x: point.x,
    z: point.z,
  }));
}

function computePathMetrics(path) {
  const cumulative = [0];
  for (let index = 1; index < path.length; index += 1) {
    const prev = path[index - 1];
    const current = path[index];
    cumulative.push(cumulative[index - 1] + Math.hypot(current.x - prev.x, current.z - prev.z));
  }
  return {
    cumulative,
    totalLength: cumulative[cumulative.length - 1] ?? 0,
  };
}

function closestPointOnSegment(point, a, b) {
  const abX = b.x - a.x;
  const abZ = b.z - a.z;
  const abLenSq = abX * abX + abZ * abZ;
  if (abLenSq < 1e-8) {
    return { point: a, t: 0, distance: Math.hypot(point.x - a.x, point.z - a.z) };
  }

  const apX = point.x - a.x;
  const apZ = point.z - a.z;
  const t = clamp((apX * abX + apZ * abZ) / abLenSq, 0, 1);
  const proj = { x: a.x + abX * t, z: a.z + abZ * t };
  return { point: proj, t, distance: Math.hypot(point.x - proj.x, point.z - proj.z) };
}

function projectPointOntoPath(point, path, metrics) {
  let best = {
    progress: 0,
    point: path[0],
    distance: Number.POSITIVE_INFINITY,
    segmentIndex: 0,
  };

  for (let index = 0; index < path.length - 1; index += 1) {
    const result = closestPointOnSegment(point, path[index], path[index + 1]);
    if (result.distance < best.distance) {
      const segmentLength = metrics.cumulative[index + 1] - metrics.cumulative[index];
      best = {
        progress: metrics.cumulative[index] + segmentLength * result.t,
        point: result.point,
        distance: result.distance,
        segmentIndex: index,
      };
    }
  }

  return best;
}

function samplePathAtProgress(path, metrics, progress) {
  if (progress <= 0) {
    return path[0];
  }
  if (progress >= metrics.totalLength) {
    return path[path.length - 1];
  }

  for (let index = 0; index < metrics.cumulative.length - 1; index += 1) {
    const startProgress = metrics.cumulative[index];
    const endProgress = metrics.cumulative[index + 1];
    if (progress <= endProgress) {
      const span = endProgress - startProgress;
      const t = span > 1e-8 ? (progress - startProgress) / span : 0;
      return {
        x: THREE.MathUtils.lerp(path[index].x, path[index + 1].x, t),
        z: THREE.MathUtils.lerp(path[index].z, path[index + 1].z, t),
      };
    }
  }

  return path[path.length - 1];
}

async function main() {
  const cfg = await loadConfig();
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
    100
  );

  const { scene, scanRing, obstacleBounds } = createScene(cfg);
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

  const halfWidth = cfg.map.width_m / 2 - 0.4;
  const halfHeight = cfg.map.height_m / 2 - 0.4;
  const moveDirection = new THREE.Vector3();
  const chaseOffset = new THREE.Vector3();
  const chaseLookAt = new THREE.Vector3();
  const chasePosition = new THREE.Vector3();
  const upAxis = new THREE.Vector3(0, 1, 0);
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  const floorPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const planeHit = new THREE.Vector3();
  const orbitState = {
    azimuth: Math.PI,
    polar: 0.92,
    distance: 3.5,
    dragging: false,
    pointerId: null,
    lastX: 0,
    lastY: 0,
  };
  const autoCommandState = {
    forward: 0,
    lateral: 0,
    turn: 0,
    speed: 0,
  };
  const plannerState = {
    mode: null,
    start: null,
    goal: null,
    path: null,
    smoothPath: null,
    pathMetrics: null,
    autoActive: false,
    pathProgress: 0,
  };
  const startMarker = createMarker(0x4de18b, 0.1, 0.12);
  const goalMarker = createMarker(0xff7a7a, 0.1, 0.12);
  const routeGroup = new THREE.Group();
  scene.add(startMarker);
  scene.add(goalMarker);
  scene.add(routeGroup);

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
    updateHud(cfg, activeRobot.label);
    updatePlannerStatus(`${activeRobot.label} selected.`);
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

  function planCurrentPath() {
    if (!plannerState.start || !plannerState.goal) {
      updatePlannerStatus("Set both start and goal first.");
      return;
    }

    const inflatedBounds = inflateBounds(obstacleBounds, robotCollisionRadius + 0.03);
    if (
      inflatedBounds.some((rect) => pointInRect(plannerState.start, rect)) ||
      inflatedBounds.some((rect) => pointInRect(plannerState.goal, rect))
    ) {
      updatePlannerStatus("Start or goal is inside an obstacle margin.");
      return;
    }

    const { nodes, graph } = buildVisibilityGraph(
      plannerState.start,
      plannerState.goal,
      inflatedBounds,
      cfg.map.width_m,
      cfg.map.height_m
    );
    const rawPath = dijkstraShortestPath(nodes, graph, 0, 1);
    if (!rawPath) {
      plannerState.path = null;
      plannerState.smoothPath = null;
      plannerState.pathMetrics = null;
      plannerState.autoActive = false;
      clearRouteRender();
      updatePlannerStatus("No collision-free path found.");
      return;
    }

    plannerState.path = prunePath(rawPath, inflatedBounds);
    plannerState.smoothPath = buildSmoothPath(plannerState.path);
    plannerState.pathMetrics = computePathMetrics(plannerState.smoothPath);
    plannerState.autoActive = false;
    clearRouteRender();
    routeGroup.add(buildPathRenderables(plannerState.path));

    const totalLength = plannerState.path.reduce((sum, point, index) => {
      if (index === 0) {
        return sum;
      }
      return sum + Math.hypot(point.x - plannerState.path[index - 1].x, point.z - plannerState.path[index - 1].z);
    }, 0);
    updatePlannerStatus(`Path planned. ${plannerState.path.length} nodes / ${totalLength.toFixed(2)}m`);
  }

  function beginAutoNav() {
    if (!plannerState.smoothPath || plannerState.smoothPath.length < 2) {
      updatePlannerStatus("Plan a path before starting auto nav.");
      return;
    }

    const projection = projectPointOntoPath(
      { x: motionState.x, z: motionState.z },
      plannerState.smoothPath,
      plannerState.pathMetrics
    );
    plannerState.autoActive = true;
    plannerState.pathProgress = projection.progress;
    autoCommandState.forward = 0;
    autoCommandState.lateral = 0;
    autoCommandState.turn = 0;
    autoCommandState.speed = 0;
    updatePlannerStatus("Auto navigation running.");
    document.getElementById("auto-nav-btn").textContent = "Stop Auto";
    document.getElementById("auto-nav-btn").classList.add("is-active");
  }

  function stopAutoNav(statusText) {
    plannerState.autoActive = false;
    plannerState.pathProgress = 0;
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
    updateHud(cfg);
  }

  function onKeyChange(event, isPressed) {
    const key = event.code;

    if (
      [
        "KeyW",
        "KeyA",
        "KeyS",
        "KeyD",
        "ArrowLeft",
        "ArrowRight",
        "Space",
        "ControlLeft",
        "ControlRight",
        "ShiftLeft",
        "ShiftRight",
      ].includes(key)
    ) {
      event.preventDefault();
    }

    if (key === "KeyW") inputState.forward = isPressed;
    if (key === "KeyS") inputState.backward = isPressed;
    if (key === "KeyA") inputState.left = isPressed;
    if (key === "KeyD") inputState.right = isPressed;
    if (key === "ArrowLeft") inputState.turnLeft = isPressed;
    if (key === "ArrowRight") inputState.turnRight = isPressed;
    if (key === "ControlLeft" || key === "ControlRight") inputState.sprint = isPressed;
    if (key === "ShiftLeft" || key === "ShiftRight") inputState.sneak = isPressed;

    if (key === "Space" && isPressed && motionState.isGrounded && activeRobot.canJump) {
      motionState.velocityY = jumpSpeed;
      motionState.isGrounded = false;
    }
  }

  window.addEventListener("keydown", (event) => onKeyChange(event, true));
  window.addEventListener("keyup", (event) => onKeyChange(event, false));

  function onPointerDown(event) {
    if (plannerState.mode) {
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);

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
        plannerState.path = null;
        plannerState.smoothPath = null;
        plannerState.pathMetrics = null;
        clearRouteRender();
        stopAutoNav();
        setPlannerMode(null);
      }
      return;
    }

    orbitState.dragging = true;
    orbitState.pointerId = event.pointerId;
    orbitState.lastX = event.clientX;
    orbitState.lastY = event.clientY;
    renderer.domElement.setPointerCapture(event.pointerId);
  }

  function onPointerMove(event) {
    if (!orbitState.dragging || orbitState.pointerId !== event.pointerId) {
      return;
    }

    const deltaX = event.clientX - orbitState.lastX;
    const deltaY = event.clientY - orbitState.lastY;
    orbitState.lastX = event.clientX;
    orbitState.lastY = event.clientY;

    orbitState.azimuth -= deltaX * 0.01;
    orbitState.polar = clamp(orbitState.polar + deltaY * 0.008, 0.35, 1.35);
  }

  function onPointerUp(event) {
    if (orbitState.pointerId === event.pointerId) {
      orbitState.dragging = false;
      renderer.domElement.releasePointerCapture(event.pointerId);
      orbitState.pointerId = null;
    }
  }

  function onWheel(event) {
    event.preventDefault();
    orbitState.distance = clamp(orbitState.distance + event.deltaY * 0.0035, 2.1, 6.5);
  }

  renderer.domElement.addEventListener("pointerdown", onPointerDown);
  renderer.domElement.addEventListener("pointermove", onPointerMove);
  renderer.domElement.addEventListener("pointerup", onPointerUp);
  renderer.domElement.addEventListener("pointercancel", onPointerUp);
  renderer.domElement.addEventListener("wheel", onWheel, { passive: false });
  document.getElementById("set-start-btn").addEventListener("click", () => {
    const nextMode = plannerState.mode === "start" ? null : "start";
    setPlannerMode(nextMode);
    updatePlannerStatus(nextMode ? "Click the map to place the start point." : "Select start and goal points.");
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
  robots.turtlebot3.groundOffset = calibrateGenericGroundOffset(turtlebot3Rig.root);
  motionState.groundOffset = activeRobot.groundOffset;
  turtlebot3Rig.root.visible = false;

  function animate() {
    const delta = Math.min(clock.getDelta(), 0.05);
    const elapsed = clock.elapsedTime;
    const pulse = 1 + Math.sin(elapsed * 1.8) * 0.035;

    let turnInput = (inputState.turnLeft ? 1 : 0) - (inputState.turnRight ? 1 : 0);
    let forwardInput = (inputState.forward ? 1 : 0) - (inputState.backward ? 1 : 0);
    let lateralInput = (inputState.right ? 1 : 0) - (inputState.left ? 1 : 0);
    let currentMoveSpeed = walkSpeed;
    if (inputState.sneak) {
      currentMoveSpeed = sneakSpeed;
    } else if (inputState.sprint && inputState.forward) {
      currentMoveSpeed = sprintSpeed;
    }

    if (plannerState.autoActive && plannerState.smoothPath && plannerState.pathMetrics) {
      const currentPoint = { x: motionState.x, z: motionState.z };
      const projection = projectPointOntoPath(currentPoint, plannerState.smoothPath, plannerState.pathMetrics);
      plannerState.pathProgress = Math.max(plannerState.pathProgress, projection.progress);

      const remaining = plannerState.pathMetrics.totalLength - plannerState.pathProgress;
      const goalPoint = plannerState.smoothPath[plannerState.smoothPath.length - 1];
      const goalDistance = Math.hypot(goalPoint.x - motionState.x, goalPoint.z - motionState.z);

      if (remaining < 0.08 && goalDistance < 0.12) {
        plannerState.start = worldToMapPoint(go2Rig.root.position);
        setMarkerPosition(startMarker, plannerState.start, motionState.supportY);
        stopAutoNav("Goal reached.");
      } else {
        const lookAhead = clamp(0.34 + projection.distance * 1.15, 0.28, 0.7);
        const targetProgress = clamp(
          plannerState.pathProgress + lookAhead,
          0,
          plannerState.pathMetrics.totalLength
        );
        const targetPoint = samplePathAtProgress(
          plannerState.smoothPath,
          plannerState.pathMetrics,
          targetProgress
        );
        const toTargetX = targetPoint.x - motionState.x;
        const toTargetZ = targetPoint.z - motionState.z;
        const targetYaw = Math.atan2(-toTargetZ, toTargetX);
        const yawDelta =
          THREE.MathUtils.euclideanModulo(targetYaw - motionState.yaw + Math.PI, Math.PI * 2) - Math.PI;

        const desiredTurn = clamp(yawDelta * 1.5, -0.62, 0.62);
        const headingAbs = Math.abs(yawDelta);
        let desiredForward = 0.44;
        if (headingAbs > 0.75) {
          desiredForward = 0;
        } else if (headingAbs > 0.4) {
          desiredForward = 0.12;
        }

        const speedScale =
          goalDistance < 0.45
            ? clamp(goalDistance / 0.45, 0.16, 1)
            : clamp(1 - projection.distance * 0.95, 0.35, 1);
        const desiredSpeed = autoNavSpeed * speedScale;

        const maxForwardDelta = 0.8 * delta;
        const maxTurnDelta = 1.15 * delta;
        const maxSpeedDelta = 0.52 * delta;

        autoCommandState.forward = moveTowards(autoCommandState.forward, desiredForward, maxForwardDelta);
        autoCommandState.lateral = moveTowards(autoCommandState.lateral, 0, maxForwardDelta);
        autoCommandState.turn = moveTowards(autoCommandState.turn, desiredTurn, maxTurnDelta);
        autoCommandState.speed = moveTowards(autoCommandState.speed, desiredSpeed, maxSpeedDelta);

        turnInput = autoCommandState.turn;
        forwardInput = autoCommandState.forward;
        lateralInput = autoCommandState.lateral;
        currentMoveSpeed = autoCommandState.speed;
      }
    }

    if (activeRobot.type === "turtlebot3") {
      lateralInput = 0;
    }

    motionState.yaw += turnInput * turnSpeed * delta;

    moveDirection.set(
      Math.cos(motionState.yaw) * forwardInput + Math.sin(motionState.yaw) * lateralInput,
      0,
      -Math.sin(motionState.yaw) * forwardInput + Math.cos(motionState.yaw) * lateralInput
    );

    let normalizedSpeed = 0;

    if (moveDirection.lengthSq() > 0) {
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
    const idleBob = motionState.isGrounded ? Math.sin(elapsed * 1.2) * (inputState.sneak ? 0.007 : 0.015) : 0;
    activeRobot.root.position.set(
      motionState.x,
      motionState.groundOffset + motionState.supportY + motionState.jumpY + idleBob + crouchOffset,
      motionState.z
    );
    activeRobot.root.rotation.y = motionState.yaw;

    if (activeRobot.type === "go2") {
      updateGo2Pose(go2Rig, motionState, elapsed, {
        speed: normalizedSpeed,
        cadence: THREE.MathUtils.lerp(3.2, 8.8, Math.max(normalizedSpeed, Math.abs(turnInput) * 0.8)),
        forward: forwardInput,
        lateral: lateralInput,
        turn: turnInput,
        forwardAmount: Math.abs(forwardInput) * normalizedSpeed,
        lateralAmount: Math.abs(lateralInput) * Math.max(normalizedSpeed, 0.45),
        turnAmount: Math.abs(turnInput) * Math.max(0.5, 1 - normalizedSpeed * 0.25),
        sneak: inputState.sneak,
      });
    } else {
      const linearVelocity = currentMoveSpeed * forwardInput;
      const yawRate = turnInput * turnSpeed;
      const leftWheelLinear = linearVelocity - yawRate * (turtlebot3Spec.wheelSeparation * 0.5);
      const rightWheelLinear = linearVelocity + yawRate * (turtlebot3Spec.wheelSeparation * 0.5);
      turtlebot3Rig.wheelLeftPivot.rotation.z -= (leftWheelLinear / turtlebot3Spec.wheelRadius) * delta;
      turtlebot3Rig.wheelRightPivot.rotation.z -= (rightWheelLinear / turtlebot3Spec.wheelRadius) * delta;
    }

    scanRing.scale.setScalar(pulse);
    scanRing.position.set(activeRobot.root.position.x, 0, activeRobot.root.position.z);

    const cameraYaw = motionState.yaw + orbitState.azimuth;
    const horizontalRadius = Math.cos(orbitState.polar) * orbitState.distance;
    const verticalOffset = Math.sin(orbitState.polar) * orbitState.distance;
    chaseOffset.set(-horizontalRadius, verticalOffset, 0);
    chaseOffset.applyAxisAngle(upAxis, cameraYaw);
    chasePosition.copy(activeRobot.root.position).add(chaseOffset);
    camera.position.lerp(chasePosition, 0.12);

    chaseLookAt.set(
      activeRobot.root.position.x + Math.cos(motionState.yaw) * 0.55,
      0.42 + motionState.jumpY * 0.25 + crouchOffset * 0.3,
      activeRobot.root.position.z - Math.sin(motionState.yaw) * 0.55
    );
    camera.lookAt(chaseLookAt);

    syncPoseToHud();

    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  window.addEventListener("resize", resize);
  animate();
}

main();
