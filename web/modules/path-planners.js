import * as THREE from "three";

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function inflateBounds(bounds, margin) {
  return bounds.map((box) => ({
    minX: box.minX - margin,
    maxX: box.maxX + margin,
    minZ: box.minZ - margin,
    maxZ: box.maxZ + margin,
  }));
}

export function pointInRect(point, rect) {
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

function buildSmoothPath(path) {
  if (!path || path.length < 2) {
    return path;
  }

  if (path.length < 3) {
    return path.map((point) => ({ x: point.x, z: point.z }));
  }

  const curve = new THREE.CatmullRomCurve3(
    path.map((point) => new THREE.Vector3(point.x, 0, point.z)),
    false,
    "centripetal",
    0.15
  );

  return curve.getPoints(Math.max(32, path.length * 18)).map((point) => ({
    x: point.x,
    z: point.z,
  }));
}

export function computePathMetrics(path) {
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

export function projectPointOntoPath(point, path, metrics) {
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

export function samplePathAtProgress(path, metrics, progress) {
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

export function planWithVisibilityGraph({
  start,
  goal,
  obstacleBounds,
  mapWidth,
  mapHeight,
  clearance,
}) {
  const inflatedBounds = inflateBounds(obstacleBounds, clearance);
  if (
    inflatedBounds.some((rect) => pointInRect(start, rect)) ||
    inflatedBounds.some((rect) => pointInRect(goal, rect))
  ) {
    return {
      ok: false,
      reason: "blocked-endpoint",
      inflatedBounds,
    };
  }

  const { nodes, graph } = buildVisibilityGraph(start, goal, inflatedBounds, mapWidth, mapHeight);
  const rawPath = dijkstraShortestPath(nodes, graph, 0, 1);
  if (!rawPath) {
    return {
      ok: false,
      reason: "no-path",
      inflatedBounds,
      debugData: {
        start,
        goal,
        nodes,
        graph,
        rawPath: null,
        path: null,
        obstacleBounds,
        inflatedBounds,
      },
    };
  }

  const path = prunePath(rawPath, inflatedBounds);
  const polylineMetrics = computePathMetrics(path);
  const smoothPath = buildSmoothPath(path);
  const smoothPathMetrics = computePathMetrics(smoothPath);
  const totalLength = path.reduce((sum, point, index) => {
    if (index === 0) {
      return sum;
    }
    return sum + Math.hypot(point.x - path[index - 1].x, point.z - path[index - 1].z);
  }, 0);

  return {
    ok: true,
    path,
    polylineMetrics,
    smoothPath,
    smoothPathMetrics,
    totalLength,
    debugData: {
      start,
      goal,
      nodes,
      graph,
      rawPath,
      path,
      obstacleBounds,
      inflatedBounds,
    },
  };
}

export const plannerModules = {
  visibilityGraph: {
    key: "visibilityGraph",
    label: "Visibility Graph",
    plan: planWithVisibilityGraph,
  },
};
