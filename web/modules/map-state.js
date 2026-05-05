const mapStorageKey = "robot-nav-viewer.map-state.v1";

export const initialObstacles = [
  { x: 2.0, y: 1.0, w: 3.0, d: 0.3, h: 0.45 },
  { x: 6.0, y: 3.0, w: 0.4, d: 2.5, h: 0.8 },
  { x: 1.0, y: 5.5, w: 2.8, d: 0.4, h: 0.55 },
  { x: 7.4, y: 0.8, w: 0.55, d: 0.6, h: 0.12 },
  { x: 7.95, y: 0.8, w: 0.55, d: 0.6, h: 0.24 },
  { x: 8.5, y: 0.8, w: 0.55, d: 0.6, h: 0.36 },
  { x: 9.05, y: 0.8, w: 0.55, d: 0.6, h: 0.48 },
];

export function hydrateObstacleList(obstacles, nextIdStart = 1) {
  let nextId = nextIdStart;
  const usedIds = new Set();
  const hydrated = obstacles.map((obstacle) => {
    let assignedId = Number(obstacle.id);
    if (!Number.isInteger(assignedId) || usedIds.has(assignedId) || assignedId < 1) {
      while (usedIds.has(nextId)) {
        nextId += 1;
      }
      assignedId = nextId++;
    }
    usedIds.add(assignedId);
    nextId = Math.max(nextId, assignedId + 1);
    return {
      ...obstacle,
      id: assignedId,
      elevation: obstacle.elevation ?? 0,
    };
  });
  return {
    nextId,
    obstacles: hydrated,
  };
}

export function loadStoredMapState() {
  try {
    const raw = window.localStorage.getItem(mapStorageKey);
    if (!raw) {
      return null;
    }
    return JSON.parse(raw);
  } catch {
    return null;
  }
}

export function saveStoredMapState(cfg, mapState) {
  try {
    window.localStorage.setItem(
      mapStorageKey,
      JSON.stringify({
        map: cfg.map,
        go2_pose: cfg.go2_pose,
        obstacles: mapState.obstacles,
      })
    );
  } catch {
    // Ignore localStorage errors and keep runtime state alive.
  }
}

export function applyStoredMapState(cfg, storedMapState) {
  if (storedMapState?.map) {
    cfg.map = {
      width_m: Number(storedMapState.map.width_m ?? cfg.map.width_m),
      height_m: Number(storedMapState.map.height_m ?? cfg.map.height_m),
    };
  }

  if (storedMapState?.go2_pose) {
    cfg.go2_pose = {
      x_m: Number(storedMapState.go2_pose.x_m ?? cfg.go2_pose.x_m),
      y_m: Number(storedMapState.go2_pose.y_m ?? cfg.go2_pose.y_m),
      yaw_deg: Number(storedMapState.go2_pose.yaw_deg ?? cfg.go2_pose.yaw_deg),
    };
  }

  return cfg;
}

export function createRuntimeMapState(storedMapState) {
  const hydratedObstacles = hydrateObstacleList(storedMapState?.obstacles ?? initialObstacles);
  return {
    nextObstacleId: hydratedObstacles.nextId,
    obstacles: hydratedObstacles.obstacles,
  };
}
