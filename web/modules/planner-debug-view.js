export function showPlannerDebugPanel(visible) {
  const panel = document.getElementById("planner-debug");
  if (!panel) {
    return;
  }
  panel.classList.toggle("is-visible", visible);
  panel.setAttribute("aria-hidden", visible ? "false" : "true");
}

export function drawPlannerDebugView(debugData, cfg, robotLabel) {
  const canvas = document.getElementById("planner-debug-canvas");
  if (!canvas || !debugData) {
    return;
  }

  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#07101a";
  ctx.fillRect(0, 0, width, height);

  const pad = 14;
  const scale = Math.min(
    (width - pad * 2) / cfg.map.width_m,
    (height - pad * 2) / cfg.map.height_m
  );
  const mapLeft = pad;
  const mapTop = pad;

  function toCanvas(point) {
    return {
      x: mapLeft + (cfg.map.width_m / 2 - point.x) * scale,
      y: mapTop + (cfg.map.height_m / 2 - point.z) * scale,
    };
  }

  ctx.strokeStyle = "rgba(126, 208, 255, 0.18)";
  ctx.lineWidth = 1;
  for (let x = 0; x <= cfg.map.width_m; x += 1) {
    const canvasX = mapLeft + x * scale;
    ctx.beginPath();
    ctx.moveTo(canvasX, mapTop);
    ctx.lineTo(canvasX, mapTop + cfg.map.height_m * scale);
    ctx.stroke();
  }
  for (let z = 0; z <= cfg.map.height_m; z += 1) {
    const canvasY = mapTop + z * scale;
    ctx.beginPath();
    ctx.moveTo(mapLeft, canvasY);
    ctx.lineTo(mapLeft + cfg.map.width_m * scale, canvasY);
    ctx.stroke();
  }

  ctx.fillStyle = "rgba(255, 122, 26, 0.18)";
  for (const rect of debugData.inflatedBounds) {
    const p1 = toCanvas({ x: rect.maxX, z: rect.maxZ });
    const rectWidth = (rect.maxX - rect.minX) * scale;
    const rectHeight = (rect.maxZ - rect.minZ) * scale;
    ctx.fillRect(p1.x, p1.y, rectWidth, rectHeight);
  }

  ctx.fillStyle = "rgba(159, 215, 255, 0.35)";
  for (const rect of debugData.obstacleBounds) {
    const p1 = toCanvas({ x: rect.maxX, z: rect.maxZ });
    const rectWidth = (rect.maxX - rect.minX) * scale;
    const rectHeight = (rect.maxZ - rect.minZ) * scale;
    ctx.fillRect(p1.x, p1.y, rectWidth, rectHeight);
  }

  ctx.strokeStyle = "rgba(141, 152, 170, 0.26)";
  ctx.lineWidth = 1;
  for (let index = 0; index < debugData.graph.length; index += 1) {
    const from = toCanvas(debugData.nodes[index]);
    for (const edge of debugData.graph[index]) {
      if (edge.to <= index) {
        continue;
      }
      const to = toCanvas(debugData.nodes[edge.to]);
      ctx.beginPath();
      ctx.moveTo(from.x, from.y);
      ctx.lineTo(to.x, to.y);
      ctx.stroke();
    }
  }

  if (debugData.rawPath && debugData.rawPath.length >= 2) {
    ctx.strokeStyle = "rgba(255, 122, 26, 0.68)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    debugData.rawPath.forEach((point, index) => {
      const p = toCanvas(point);
      if (index === 0) {
        ctx.moveTo(p.x, p.y);
      } else {
        ctx.lineTo(p.x, p.y);
      }
    });
    ctx.stroke();
  }

  if (debugData.path && debugData.path.length >= 2) {
    ctx.strokeStyle = robotLabel.includes("TurtleBot3") ? "#ffb176" : "#1d8f4d";
    ctx.lineWidth = 3;
    ctx.beginPath();
    debugData.path.forEach((point, index) => {
      const p = toCanvas(point);
      if (index === 0) {
        ctx.moveTo(p.x, p.y);
      } else {
        ctx.lineTo(p.x, p.y);
      }
    });
    ctx.stroke();
  }

  ctx.fillStyle = "#4de18b";
  const start = toCanvas(debugData.start);
  ctx.beginPath();
  ctx.arc(start.x, start.y, 5, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#ff7a7a";
  const goal = toCanvas(debugData.goal);
  ctx.beginPath();
  ctx.arc(goal.x, goal.y, 5, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = "#d8e0eb";
  for (const node of debugData.nodes) {
    const p = toCanvas(node);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
    ctx.fill();
  }

  const edgeCount = debugData.graph.reduce((sum, edges) => sum + edges.length, 0) / 2;
  document.querySelector('[data-role="debug-robot"]').textContent = robotLabel;
  document.querySelector('[data-role="debug-nodes"]').textContent = String(debugData.nodes.length);
  document.querySelector('[data-role="debug-edges"]').textContent = String(edgeCount);
  document.querySelector('[data-role="debug-raw"]').textContent = String(debugData.rawPath?.length ?? 0);
  document.querySelector('[data-role="debug-final"]').textContent = String(debugData.path?.length ?? 0);
  showPlannerDebugPanel(true);
}
