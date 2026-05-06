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

function lerp(a, b, t) {
  return a + (b - a) * t;
}

function defaultOptions() {
  return {
    gridStep: 1.25,
    maxVisited: 1800,
    maxUpStep: 0.68,
    maxDownStep: 0.95,
    searchMarginMin: 10,
    searchMarginMax: 28,
    clearance: 0.34,
    maxLineCheckDistance: 8,
    maxCheapLineDistance: 80,
    flatHeightTolerance: 0.22,
    flatGridStep: 1.65,
    flatMaxVisited: 1400,
    shortStepDistance: 5,
    stairLineDistance: 18,
  };
}

// 3D Tiles planning is intentionally staged from cheap to expensive:
// direct stair checks first, flat 2D-like grid next, full 3D A* only as fallback.
function pushHeap(heap, item, scoreOf) {
  heap.push(item);
  let index = heap.length - 1;
  while (index > 0) {
    const parent = Math.floor((index - 1) / 2);
    if (scoreOf(heap[parent]) <= scoreOf(item)) {
      break;
    }
    heap[index] = heap[parent];
    index = parent;
  }
  heap[index] = item;
}

function popHeap(heap, scoreOf) {
  if (heap.length === 0) {
    return null;
  }
  const root = heap[0];
  const last = heap.pop();
  if (heap.length > 0) {
    let index = 0;
    while (true) {
      const left = index * 2 + 1;
      const right = left + 1;
      if (left >= heap.length) {
        break;
      }
      const child =
        right < heap.length && scoreOf(heap[right]) < scoreOf(heap[left])
          ? right
          : left;
      if (scoreOf(last) <= scoreOf(heap[child])) {
        break;
      }
      heap[index] = heap[child];
      index = child;
    }
    heap[index] = last;
  }
  return root;
}

export function planTilesRoute({
  start,
  goal,
  findSupportY,
  isHorizontalBlocked,
  robotRadius,
  options = {},
}) {
  const config = { ...defaultOptions(), ...options };
  if (!start || !goal) {
    return { ok: false, reason: "missing-endpoints" };
  }

  const startNode = {
    x: start.x,
    z: start.z,
    y: start.y ?? findSupportY(start.x, start.z, 0),
  };
  const goalNode = {
    x: goal.x,
    z: goal.z,
    y: goal.y ?? findSupportY(goal.x, goal.z, startNode.y ?? 0),
  };

  if (startNode.y === null || goalNode.y === null) {
    return { ok: false, reason: "missing-surface" };
  }

  const clearanceRadius = Math.max(robotRadius + 0.1, config.clearance);
  const visitedNodes = [];
  const supportCache = new Map();
  const footprintCache = new Map();
  const segmentCache = new Map();
  const gridKeyFor = (x, z) => `${Math.round(x / config.gridStep)}:${Math.round(z / config.gridStep)}`;

  function cachedSupportY(x, z, baseY) {
    const key = `${gridKeyFor(x, z)}:${Math.round(baseY * 5)}`;
    if (supportCache.has(key)) {
      return supportCache.get(key);
    }
    const supportY = findSupportY(x, z, baseY);
    supportCache.set(key, supportY);
    return supportY;
  }

  function hasFootprintSupport(point, baseY) {
    const offsets = [
      [0, 0],
      [clearanceRadius, 0],
      [-clearanceRadius, 0],
      [0, clearanceRadius],
      [0, -clearanceRadius],
      [clearanceRadius * 0.7, clearanceRadius * 0.7],
      [clearanceRadius * 0.7, -clearanceRadius * 0.7],
      [-clearanceRadius * 0.7, clearanceRadius * 0.7],
      [-clearanceRadius * 0.7, -clearanceRadius * 0.7],
    ];
    let bestY = null;
    for (const [ox, oz] of offsets) {
      const supportY = cachedSupportY(point.x + ox, point.z + oz, baseY);
      if (supportY === null) {
        return null;
      }
      if (supportY - baseY > config.maxUpStep + 0.03 || baseY - supportY > config.maxDownStep) {
        return null;
      }
      bestY = bestY === null ? supportY : Math.max(bestY, supportY);
    }
    return bestY;
  }

  function canTraverseSegment(a, b, sampleStep = config.gridStep * 0.45) {
    const cacheKey = `${gridKeyFor(a.x, a.z)}>${gridKeyFor(b.x, b.z)}:${Math.round((a.y ?? 0) * 4)}`;
    if (segmentCache.has(cacheKey)) {
      return segmentCache.get(cacheKey);
    }
    const distance = Math.hypot(b.x - a.x, b.z - a.z);
    const samples = Math.max(1, Math.ceil(distance / sampleStep));
    const dirX = distance > 1e-6 ? (b.x - a.x) / distance : 1;
    const dirZ = distance > 1e-6 ? (b.z - a.z) / distance : 0;
    const sideX = -dirZ;
    const sideZ = dirX;
    let current = { x: a.x, z: a.z, y: a.y ?? 0 };

    for (let index = 1; index <= samples; index += 1) {
      const t = index / samples;
      const next = {
        x: lerp(a.x, b.x, t),
        z: lerp(a.z, b.z, t),
      };
      const centerY = cachedSupportY(next.x, next.z, current.y);
      if (centerY === null) {
        segmentCache.set(cacheKey, null);
        return null;
      }
      if (centerY - current.y > config.maxUpStep + 0.03 || current.y - centerY > config.maxDownStep) {
        segmentCache.set(cacheKey, null);
        return null;
      }
      const footprintY = hasFootprintSupport(next, current.y);
      if (footprintY === null) {
        segmentCache.set(cacheKey, null);
        return null;
      }
      const sideChecks = [
        [0, 0],
        [sideX * clearanceRadius, sideZ * clearanceRadius],
        [-sideX * clearanceRadius, -sideZ * clearanceRadius],
      ];
      for (const [ox, oz] of sideChecks) {
        if (
          isHorizontalBlocked(
            current.x + ox,
            current.z + oz,
            next.x + ox,
            next.z + oz,
            clearanceRadius,
            current.y
          )
        ) {
          segmentCache.set(cacheKey, null);
          return null;
        }
      }
      current = { x: next.x, z: next.z, y: Math.max(centerY, footprintY) };
    }

    const result = { x: b.x, z: b.z, y: current.y };
    segmentCache.set(cacheKey, result);
    return result;
  }

  function canTraverseCheapSegment(a, b) {
    const distance = Math.hypot(b.x - a.x, b.z - a.z);
    if (distance > config.maxCheapLineDistance) {
      return null;
    }
    return canTraverseSegment(a, b, Math.max(0.75, config.gridStep * 0.72));
  }

  function canTraverseFlatSegment(a, b, sampleStep = config.flatGridStep * 0.65) {
    const distance = Math.hypot(b.x - a.x, b.z - a.z);
    const samples = Math.max(1, Math.ceil(distance / sampleStep));
    const dirX = distance > 1e-6 ? (b.x - a.x) / distance : 1;
    const dirZ = distance > 1e-6 ? (b.z - a.z) / distance : 0;
    const sideX = -dirZ;
    const sideZ = dirX;
    let current = { x: a.x, z: a.z, y: a.y ?? startNode.y };

    for (let index = 1; index <= samples; index += 1) {
      const t = index / samples;
      const next = {
        x: lerp(a.x, b.x, t),
        z: lerp(a.z, b.z, t),
      };
      const centerY = cachedSupportY(next.x, next.z, current.y);
      if (centerY === null || Math.abs(centerY - startNode.y) > config.flatHeightTolerance) {
        return null;
      }
      const sideChecks = [
        [0, 0],
        [sideX * clearanceRadius, sideZ * clearanceRadius],
        [-sideX * clearanceRadius, -sideZ * clearanceRadius],
      ];
      for (const [ox, oz] of sideChecks) {
        if (
          isHorizontalBlocked(
            current.x + ox,
            current.z + oz,
            next.x + ox,
            next.z + oz,
            clearanceRadius,
            current.y
          )
        ) {
          return null;
        }
      }
      current = { x: next.x, z: next.z, y: centerY };
    }

    return { x: b.x, z: b.z, y: current.y };
  }

  function canTraverseShortStep(a, b) {
    const distance = Math.hypot(b.x - a.x, b.z - a.z);
    const heightDelta = b.y - a.y;
    if (
      distance > config.shortStepDistance ||
      heightDelta > config.maxUpStep + 0.05 ||
      heightDelta < -config.maxDownStep
    ) {
      return null;
    }

    const samples = Math.max(2, Math.ceil(distance / 0.55));
    let current = { x: a.x, z: a.z, y: a.y };
    for (let index = 1; index <= samples; index += 1) {
      const t = index / samples;
      const next = {
        x: lerp(a.x, b.x, t),
        z: lerp(a.z, b.z, t),
      };
      const expectedY = lerp(a.y, b.y, t);
      const supportY = cachedSupportY(next.x, next.z, expectedY);
      if (supportY === null) {
        return null;
      }
      if (supportY - current.y > config.maxUpStep + 0.08 || current.y - supportY > config.maxDownStep) {
        return null;
      }
      const rayMinHeight = supportY > current.y ? Math.min(0.88, supportY - current.y + 0.16) : 0.18;
      if (isHorizontalBlocked(current.x, current.z, next.x, next.z, clearanceRadius, current.y, rayMinHeight)) {
        return null;
      }
      current = { x: next.x, z: next.z, y: supportY };
    }

    return { x: b.x, z: b.z, y: current.y };
  }

  function canTraverseStairLine(a, b) {
    // Stair treads often expose vertical riser faces. When each local rise is
    // climbable, treat those risers as steps instead of walls so straight stairs
    // do not detour sideways.
    const distance = Math.hypot(b.x - a.x, b.z - a.z);
    if (distance > config.stairLineDistance) {
      return null;
    }

    const samples = Math.max(3, Math.ceil(distance / 0.42));
    const totalHeightDelta = b.y - a.y;
    let current = { x: a.x, z: a.z, y: a.y };
    let maxRise = 0;
    let maxDrop = 0;
    let heightTravel = 0;
    let validSamples = 0;

    for (let index = 1; index <= samples; index += 1) {
      const t = index / samples;
      const next = {
        x: lerp(a.x, b.x, t),
        z: lerp(a.z, b.z, t),
      };
      const expectedY = lerp(a.y, b.y, t);
      const supportY = cachedSupportY(next.x, next.z, expectedY);
      if (supportY === null) {
        return null;
      }

      const stepDelta = supportY - current.y;
      maxRise = Math.max(maxRise, stepDelta);
      maxDrop = Math.max(maxDrop, -stepDelta);
      heightTravel += Math.abs(stepDelta);
      validSamples += 1;
      if (stepDelta > config.maxUpStep + 0.08 || -stepDelta > config.maxDownStep) {
        return null;
      }

      const rayMinHeight = stepDelta > 0 ? Math.min(1.05, stepDelta + 0.35) : 0.22;
      const shouldIgnoreRiserWall = stepDelta > 0.03 && distance <= config.stairLineDistance;
      if (
        !shouldIgnoreRiserWall &&
        isHorizontalBlocked(current.x, current.z, next.x, next.z, clearanceRadius, current.y, rayMinHeight)
      ) {
        return null;
      }
      current = { x: next.x, z: next.z, y: supportY };
    }

    const heightMismatch = Math.abs(current.y - b.y);
    const stairOrRampLike = heightTravel >= Math.abs(totalHeightDelta) * 0.45 || validSamples > 2;
    if (
      heightMismatch > config.maxUpStep * 1.75 ||
      maxRise > config.maxUpStep + 0.08 ||
      maxDrop > config.maxDownStep ||
      !stairOrRampLike
    ) {
      return null;
    }

    return { x: b.x, z: b.z, y: current.y };
  }

  function simplifyPath(path) {
    if (!path || path.length <= 2) {
      return path;
    }

    const simplified = [path[0]];
    let anchorIndex = 0;
    while (anchorIndex < path.length - 1) {
      let bestIndex = anchorIndex + 1;
      let bestPoint = path[bestIndex];
      for (let candidate = path.length - 1; candidate > anchorIndex + 1; candidate -= 1) {
        const reachable = canTraverseSegment(path[anchorIndex], path[candidate]);
        if (reachable) {
          bestIndex = candidate;
          bestPoint = { ...path[candidate], y: reachable.y };
          break;
        }
      }
      simplified.push(bestPoint);
      anchorIndex = bestIndex;
    }
    return simplified;
  }

  if (hasFootprintSupport(startNode, startNode.y) === null || hasFootprintSupport(goalNode, goalNode.y) === null) {
    const shortStep = canTraverseShortStep(startNode, goalNode);
    const stairLine = shortStep ? null : canTraverseStairLine(startNode, goalNode);
    if (!shortStep && !stairLine) {
      return { ok: false, reason: "endpoint-clearance" };
    }
    const directFallback = shortStep ?? stairLine;
    const path = [startNode, { ...goalNode, y: directFallback.y }];
    return {
      ok: true,
      path,
      polylineMetrics: computePathMetrics(path),
      smoothPath: path,
      smoothPathMetrics: computePathMetrics(path),
      totalLength: Math.hypot(goalNode.x - startNode.x, goalNode.z - startNode.z),
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes,
        rawPath: path,
        path,
        clearanceRadius,
        plannerMode: shortStep ? "short-step" : "stair-line",
      },
      visitedCount: 0,
    };
  }

  const straightDistance = Math.hypot(goalNode.x - startNode.x, goalNode.z - startNode.z);
  const shortStepDirect = canTraverseShortStep(startNode, goalNode);
  if (shortStepDirect) {
    const path = [startNode, { ...goalNode, y: shortStepDirect.y }];
    return {
      ok: true,
      path,
      polylineMetrics: computePathMetrics(path),
      smoothPath: path,
      smoothPathMetrics: computePathMetrics(path),
      totalLength: straightDistance,
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes,
        rawPath: path,
        path,
        clearanceRadius,
        plannerMode: "short-step",
      },
      visitedCount: 0,
    };
  }
  const stairLineDirect = canTraverseStairLine(startNode, goalNode);
  if (stairLineDirect) {
    const path = [startNode, { ...goalNode, y: stairLineDirect.y }];
    return {
      ok: true,
      path,
      polylineMetrics: computePathMetrics(path),
      smoothPath: path,
      smoothPathMetrics: computePathMetrics(path),
      totalLength: straightDistance,
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes,
        rawPath: path,
        path,
        clearanceRadius,
        plannerMode: "stair-line",
      },
      visitedCount: 0,
    };
  }
  const direct = canTraverseCheapSegment(startNode, goalNode);
  if (direct) {
    const path = [startNode, { ...goalNode, y: direct.y }];
    return {
      ok: true,
      path,
      polylineMetrics: computePathMetrics(path),
      smoothPath: path,
      smoothPathMetrics: computePathMetrics(path),
      totalLength: Math.hypot(goalNode.x - startNode.x, goalNode.z - startNode.z),
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes,
        rawPath: path,
        path,
        clearanceRadius,
        plannerMode: "direct",
      },
      visitedCount: 0,
    };
  }

  function planFlatRoute() {
    // Same-floor routing uses a coarse occupancy grid, similar in spirit to the
    // sandbox map planner, to avoid expensive 3D ray checks during every A* step.
    if (Math.abs(goalNode.y - startNode.y) > config.flatHeightTolerance) {
      return null;
    }

    const flatStep = config.flatGridStep;
    const flatMargin = Math.min(
      config.searchMarginMax,
      Math.max(config.searchMarginMin, straightDistance * 0.22 + 6)
    );
    const flatMinX = Math.min(startNode.x, goalNode.x) - flatMargin;
    const flatMaxX = Math.max(startNode.x, goalNode.x) + flatMargin;
    const flatMinZ = Math.min(startNode.z, goalNode.z) - flatMargin;
    const flatMaxZ = Math.max(startNode.z, goalNode.z) + flatMargin;
    const flatNeighborOffsets = [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
      [1, 1],
      [1, -1],
      [-1, 1],
      [-1, -1],
    ];
    const flatNodes = new Map();
    const flatOpen = [];
    const flatClosed = new Set();
    const flatVisitedNodes = [];
    const flatWalkableCache = new Map();
    const flatGridKey = (x, z) => `${Math.round(x / flatStep)}:${Math.round(z / flatStep)}`;
    const flatHeuristic = (node) => Math.hypot(goalNode.x - node.x, goalNode.z - node.z);
    const flatStartKey = flatGridKey(startNode.x, startNode.z);
    const flatWalkable = (x, z, baseY) => {
      const key = flatGridKey(x, z);
      if (flatWalkableCache.has(key)) {
        return flatWalkableCache.get(key);
      }
      const supportY = cachedSupportY(x, z, baseY);
      const result =
        supportY !== null && Math.abs(supportY - startNode.y) <= config.flatHeightTolerance
          ? { x, z, y: supportY }
          : null;
      flatWalkableCache.set(key, result);
      return result;
    };
    const simplifyFlatPath = (path) => {
      if (!path || path.length <= 2) {
        return path;
      }
      const simplified = [path[0]];
      let anchorIndex = 0;
      while (anchorIndex < path.length - 1) {
        let bestIndex = anchorIndex + 1;
        let bestPoint = path[bestIndex];
        for (let candidate = path.length - 1; candidate > anchorIndex + 1; candidate -= 1) {
          const reachable = canTraverseFlatSegment(path[anchorIndex], path[candidate]);
          if (reachable) {
            bestIndex = candidate;
            bestPoint = { ...path[candidate], y: reachable.y };
            break;
          }
        }
        simplified.push(bestPoint);
        anchorIndex = bestIndex;
      }
      return simplified;
    };

    flatNodes.set(flatStartKey, {
      ...startNode,
      key: flatStartKey,
      parent: null,
      g: 0,
      f: flatHeuristic(startNode),
    });
    pushHeap(flatOpen, flatStartKey, (key) => flatNodes.get(key)?.f ?? Infinity);

    let flatVisitedCount = 0;
    let flatGoalKey = null;
    while (flatOpen.length > 0 && flatVisitedCount < config.flatMaxVisited) {
      const currentKey = popHeap(flatOpen, (key) => flatNodes.get(key)?.f ?? Infinity);
      if (flatClosed.has(currentKey)) {
        continue;
      }
      const current = flatNodes.get(currentKey);
      flatClosed.add(currentKey);
      flatVisitedCount += 1;
      if (flatVisitedNodes.length < 1400) {
        flatVisitedNodes.push({ x: current.x, z: current.z, y: current.y, f: current.f, g: current.g });
      }

      const goalDistance = Math.hypot(goalNode.x - current.x, goalNode.z - current.z);
      const goalReach = goalDistance <= Math.max(config.maxLineCheckDistance, flatStep * 2.2)
        ? canTraverseFlatSegment(current, goalNode)
        : null;
      if (goalReach && Math.abs(goalReach.y - startNode.y) <= config.flatHeightTolerance + 0.08) {
        flatGoalKey = "goal";
        flatNodes.set(flatGoalKey, {
          ...goalNode,
          y: goalReach.y,
          key: flatGoalKey,
          parent: currentKey,
          g: current.g + goalDistance,
          f: current.g + goalDistance,
        });
        break;
      }

      for (const [dx, dz] of flatNeighborOffsets) {
        const nx = current.x + dx * flatStep;
        const nz = current.z + dz * flatStep;
        if (nx < flatMinX || nx > flatMaxX || nz < flatMinZ || nz > flatMaxZ) {
          continue;
        }
        const key = flatGridKey(nx, nz);
        if (flatClosed.has(key)) {
          continue;
        }
        const walkable = flatWalkable(nx, nz, current.y);
        if (!walkable) {
          continue;
        }
        if (
          dx !== 0 &&
          dz !== 0 &&
          (!flatWalkable(current.x + dx * flatStep, current.z, current.y) ||
            !flatWalkable(current.x, current.z + dz * flatStep, current.y))
        ) {
          continue;
        }
        const nextPoint = walkable;
        const horizontalCost = Math.hypot(dx * flatStep, dz * flatStep);
        const diagonalCost = dx !== 0 && dz !== 0 ? 0.04 : 0;
        const nextG = current.g + horizontalCost + diagonalCost;
        const previous = flatNodes.get(key);
        if (previous && previous.g <= nextG) {
          continue;
        }
        const node = {
          ...nextPoint,
          key,
          parent: currentKey,
          g: nextG,
          f: nextG + flatHeuristic(nextPoint),
        };
        flatNodes.set(key, node);
        pushHeap(flatOpen, key, (heapKey) => flatNodes.get(heapKey)?.f ?? Infinity);
      }
    }

    if (!flatGoalKey) {
      return null;
    }

    const rawPath = [];
    for (let cursor = flatGoalKey; cursor; cursor = flatNodes.get(cursor)?.parent) {
      const node = flatNodes.get(cursor);
      if (!node) {
        break;
      }
      rawPath.push({ x: node.x, z: node.z, y: node.y });
    }
    rawPath.reverse();
    const path = simplifyFlatPath(rawPath);
    const totalLength = path.reduce((sum, point, index) => {
      if (index === 0) {
        return sum;
      }
      return sum + Math.hypot(point.x - path[index - 1].x, point.z - path[index - 1].z);
    }, 0);

    return {
      ok: true,
      path,
      polylineMetrics: computePathMetrics(path),
      smoothPath: path,
      smoothPathMetrics: computePathMetrics(path),
      totalLength,
      visitedCount: flatVisitedCount,
      rawPathLength: rawPath.length,
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes: flatVisitedNodes,
        rawPath,
        path,
        clearanceRadius,
        plannerMode: "flat",
      },
    };
  }

  const flatResult = planFlatRoute();
  if (flatResult) {
    return flatResult;
  }

  const margin = Math.min(
    config.searchMarginMax,
    Math.max(config.searchMarginMin, straightDistance * 0.35 + 10)
  );
  const minX = Math.min(startNode.x, goalNode.x) - margin;
  const maxX = Math.max(startNode.x, goalNode.x) + margin;
  const minZ = Math.min(startNode.z, goalNode.z) - margin;
  const maxZ = Math.max(startNode.z, goalNode.z) + margin;
  const step = config.gridStep;
  const neighborOffsets = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
    [1, 1],
    [1, -1],
    [-1, 1],
    [-1, -1],
  ];
  const nodes = new Map();
  const open = [];
  const closed = new Set();
  const gridKey = (x, z) => `${Math.round(x / step)}:${Math.round(z / step)}`;
  const heuristic = (node) =>
    Math.hypot(goalNode.x - node.x, goalNode.z - node.z) + Math.max(0, goalNode.y - node.y) * 1.4;

  const startKey = gridKey(startNode.x, startNode.z);
  nodes.set(startKey, { ...startNode, key: startKey, parent: null, g: 0, f: heuristic(startNode) });
  pushHeap(open, startKey, (key) => nodes.get(key)?.f ?? Infinity);

  let visitedCount = 0;
  let goalKey = null;
  while (open.length > 0 && visitedCount < config.maxVisited) {
    const currentKey = popHeap(open, (key) => nodes.get(key)?.f ?? Infinity);
    if (closed.has(currentKey)) {
      continue;
    }
    const current = nodes.get(currentKey);
    closed.add(currentKey);
    visitedCount += 1;
    if (visitedNodes.length < 1800) {
      visitedNodes.push({ x: current.x, z: current.z, y: current.y, f: current.f, g: current.g });
    }

    const goalDistance = Math.hypot(goalNode.x - current.x, goalNode.z - current.z);
    const goalReach = goalDistance <= config.maxLineCheckDistance
      ? canTraverseSegment(current, goalNode)
      : null;
    if (goalReach) {
      goalKey = "goal";
      nodes.set(goalKey, {
        ...goalNode,
        y: goalReach.y,
        key: goalKey,
        parent: currentKey,
        g: current.g + Math.hypot(goalNode.x - current.x, goalNode.z - current.z),
        f: current.g,
      });
      break;
    }

    for (const [dx, dz] of neighborOffsets) {
      const nx = current.x + dx * step;
      const nz = current.z + dz * step;
      if (nx < minX || nx > maxX || nz < minZ || nz > maxZ) {
        continue;
      }
      const supportY = cachedSupportY(nx, nz, current.y);
      if (supportY === null) {
        continue;
      }
      if (supportY - current.y > config.maxUpStep + 0.03 || current.y - supportY > config.maxDownStep) {
        continue;
      }
      const key = gridKey(nx, nz);
      if (closed.has(key)) {
        continue;
      }
      const nextPoint = { x: nx, z: nz, y: supportY };
      const footprintKey = `${key}:${Math.round(current.y * 5)}`;
      let footprintY = footprintCache.get(footprintKey);
      if (footprintY === undefined) {
        footprintY = hasFootprintSupport(nextPoint, current.y);
        footprintCache.set(footprintKey, footprintY);
      }
      if (footprintY === null) {
        continue;
      }
      if (!canTraverseSegment(current, nextPoint, config.gridStep * 0.5)) {
        continue;
      }

      const horizontalCost = Math.hypot(dx * step, dz * step);
      const verticalCost = Math.max(0, supportY - current.y) * 2.6 + Math.max(0, current.y - supportY) * 0.45;
      const turnCost = dx !== 0 && dz !== 0 ? 0.06 : 0;
      const nextG = current.g + horizontalCost + verticalCost + turnCost;
      const previous = nodes.get(key);
      if (previous && previous.g <= nextG) {
        continue;
      }
      const node = {
        ...nextPoint,
        key,
        parent: currentKey,
        g: nextG,
        f: nextG + heuristic(nextPoint),
      };
      nodes.set(key, node);
      pushHeap(open, key, (heapKey) => nodes.get(heapKey)?.f ?? Infinity);
    }
  }

  if (!goalKey) {
    return {
      ok: false,
      reason: visitedCount >= config.maxVisited ? "search-budget" : "no-tiles-path",
      visitedCount,
      debugData: {
        start: startNode,
        goal: goalNode,
        visitedNodes,
        rawPath: [],
        path: [],
        clearanceRadius,
      },
    };
  }

  const rawPath = [];
  for (let cursor = goalKey; cursor; cursor = nodes.get(cursor)?.parent) {
    const node = nodes.get(cursor);
    if (!node) {
      break;
    }
    rawPath.push({ x: node.x, z: node.z, y: node.y });
  }
  rawPath.reverse();
  const path = simplifyPath(rawPath);
  const totalLength = path.reduce((sum, point, index) => {
    if (index === 0) {
      return sum;
    }
    return sum + Math.hypot(point.x - path[index - 1].x, point.z - path[index - 1].z);
  }, 0);

  return {
    ok: true,
    path,
    polylineMetrics: computePathMetrics(path),
    smoothPath: path,
    smoothPathMetrics: computePathMetrics(path),
    totalLength,
    visitedCount,
    rawPathLength: rawPath.length,
    debugData: {
      start: startNode,
      goal: goalNode,
      visitedNodes,
      rawPath,
      path,
      clearanceRadius,
      plannerMode: "3d",
    },
  };
}
