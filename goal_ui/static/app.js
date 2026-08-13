const ui = {
  connection: document.querySelector('#connection'),
  frame: document.querySelector('#frameLabel'),
  form: document.querySelector('#goalForm'),
  x: document.querySelector('#xInput'),
  y: document.querySelector('#yInput'),
  stop: document.querySelector('#stopButton'),
  position: document.querySelector('#positionValue'),
  distance: document.querySelector('#distanceValue'),
  obstacle: document.querySelector('#obstacleValue'),
  velocity: document.querySelector('#velocityValue'),
  targetState: document.querySelector('#targetState'),
  event: document.querySelector('#eventText'),
  canvas: document.querySelector('#positionCanvas'),
  drawRoute: document.querySelector('#drawRouteButton'),
  undoRoute: document.querySelector('#undoRouteButton'),
  clearRoute: document.querySelector('#clearRouteButton'),
  zoomOut: document.querySelector('#zoomOutButton'),
  zoomIn: document.querySelector('#zoomInButton'),
  runRoute: document.querySelector('#runRouteButton'),
  dialog: document.querySelector('#confirmDialog'),
  confirmLabel: document.querySelector('#confirmLabel'),
  confirmCoordinates: document.querySelector('#confirmCoordinates'),
  cancelConfirm: document.querySelector('#cancelConfirm'),
  confirmGoal: document.querySelector('#confirmGoal'),
  toast: document.querySelector('#toast'),
};

let status = { position: null, target: null, route: [], local_plan: [], lidar_points: [] };
let pendingAction = null;
let draftRoute = [];
let drawMode = false;
let pointerDrawing = false;
let routeView = null;
let viewZoom = 1;
let toastTimer = null;
let lastStatusReceivedAt = 0;

function showToast(message, error = false) {
  clearTimeout(toastTimer);
  ui.toast.textContent = message;
  ui.toast.classList.toggle('error', error);
  ui.toast.classList.add('show');
  toastTimer = setTimeout(() => ui.toast.classList.remove('show'), 2600);
}

async function request(path, options = {}) {
  const response = await fetch(path, options);
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.message || '请求失败');
  return payload;
}

function activeRoute() {
  if (draftRoute.length) return draftRoute;
  if (!status.target_active) return [];
  return remainingRoute(status.route || [], status.position);
}

function remainingRoute(points, position) {
  if (!points.length || !position) return points.slice();
  if (points.length === 1) return points.slice();

  let closest = null;
  for (let index = 0; index < points.length - 1; index += 1) {
    const start = points[index];
    const end = points[index + 1];
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const lengthSquared = dx * dx + dy * dy;
    const ratio = lengthSquared > 0
      ? Math.max(0, Math.min(1,
        ((position.x - start.x) * dx + (position.y - start.y) * dy) / lengthSquared))
      : 0;
    const projected = { x: start.x + ratio * dx, y: start.y + ratio * dy };
    const distanceSquared = (position.x - projected.x) ** 2 + (position.y - projected.y) ** 2;
    if (!closest || distanceSquared < closest.distanceSquared) {
      closest = { segment: index, projected, distanceSquared };
    }
  }

  const result = [closest.projected, ...points.slice(closest.segment + 1)];
  return result.filter((point, index) => index === 0 ||
    Math.hypot(point.x - result[index - 1].x, point.y - result[index - 1].y) >= 0.01);
}

function routeLength(points) {
  return points.slice(1).reduce((total, point, index) =>
    total + Math.hypot(point.x - points[index].x, point.y - points[index].y), 0);
}

function simplifyRoute(points) {
  if (points.length <= 2) return points.slice();
  const result = [points[0]];
  for (let index = 1; index < points.length - 1; index += 1) {
    const previous = points[index - 1];
    const current = points[index];
    const next = points[index + 1];
    const fromKept = Math.hypot(current.x - result[result.length - 1].x, current.y - result[result.length - 1].y);
    const firstAngle = Math.atan2(current.y - previous.y, current.x - previous.x);
    const secondAngle = Math.atan2(next.y - current.y, next.x - current.x);
    const turn = Math.abs(Math.atan2(Math.sin(secondAngle - firstAngle), Math.cos(secondAngle - firstAngle)));
    if (fromKept >= 0.45 || (fromKept >= 0.15 && turn >= Math.PI / 9)) result.push(current);
  }
  const end = points[points.length - 1];
  if (Math.hypot(end.x - result[result.length - 1].x, end.y - result[result.length - 1].y) >= 0.10) {
    result.push(end);
  }
  return result;
}

function viewGeometry() {
  const center = status.position || { x: 0, y: 0 };
  return { x: center.x, y: center.y, zoom: viewZoom };
}

function drawMap() {
  const canvas = ui.canvas;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const view = viewGeometry();
  // Match autorunlida's fixed vehicle-frame viewport. A frame cannot alter
  // the scale; only the explicit zoom controls can.
  const bounds = {
    xMin: -0.72 * view.zoom, xMax: 1.78 * view.zoom,
    yMin: -2.5 * view.zoom, yMax: 2.5 * view.zoom,
  };
  const pad = { left: 36, right: 20, top: 24, bottom: 24 };
  const scale = Math.min(
    (w - pad.left - pad.right) / (bounds.yMax - bounds.yMin),
    (h - pad.top - pad.bottom) / (bounds.xMax - bounds.xMin),
  );
  const usedW = (bounds.yMax - bounds.yMin) * scale;
  const usedH = (bounds.xMax - bounds.xMin) * scale;
  const offsetX = pad.left + (w - pad.left - pad.right - usedW) / 2;
  const offsetY = pad.top + (h - pad.top - pad.bottom - usedH) / 2;
  const bodyToScreen = (forward, left) => ({
    x: offsetX + (bounds.yMax - left) * scale,
    y: offsetY + (bounds.xMax - forward) * scale,
  });
  const anchor = bodyToScreen(0, 0);
  const yaw = (status.position?.yaw || 0) * Math.PI / 180;
  const cosYaw = Math.cos(yaw);
  const sinYaw = Math.sin(yaw);
  const toScreen = (point) => {
    const dx = point.x - view.x;
    const dy = point.y - view.y;
    return bodyToScreen(cosYaw * dx + sinYaw * dy, -sinYaw * dx + cosYaw * dy);
  };

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#f8faf9';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#dce3e4';
  ctx.lineWidth = 1;
  const gridStep = view.zoom >= 2 ? 0.5 : 0.2;
  for (let forward = Math.ceil(bounds.xMin / gridStep) * gridStep;
       forward <= bounds.xMax; forward += gridStep) {
    const p = bodyToScreen(forward, 0);
    ctx.beginPath(); ctx.moveTo(offsetX, p.y); ctx.lineTo(offsetX + usedW, p.y); ctx.stroke();
  }
  for (let left = Math.ceil(bounds.yMin / gridStep) * gridStep;
       left <= bounds.yMax; left += gridStep) {
    const p = bodyToScreen(0, left);
    ctx.beginPath(); ctx.moveTo(p.x, offsetY); ctx.lineTo(p.x, offsetY + usedH); ctx.stroke();
  }
  ctx.strokeStyle = '#8e9aa2';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(anchor.x, offsetY); ctx.lineTo(anchor.x, offsetY + usedH); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(offsetX, anchor.y); ctx.lineTo(offsetX + usedW, anchor.y); ctx.stroke();
  ctx.fillStyle = '#667681';
  ctx.font = '600 15px Segoe UI';
  ctx.fillText('+X 前进', 18, 26);
  ctx.fillText('+Y 左侧', 18, 48);

  const obstacles = status.lidar_points || [];
  const lidarLive = status.lidar_fresh && status.lidar_points_current;
  ctx.save();
  ctx.globalAlpha = lidarLive ? 1 : 0.28;
  obstacles.forEach((point) => {
    // Same visualization-only self-reflection mask as autorunlida.
    if (point.x >= -0.64 && point.x <= 0.04 && Math.abs(point.y) <= 0.30) return;
    const screen = bodyToScreen(point.x, point.y);
    const detection = point.x >= 0.15 && point.x <= 1.60 && Math.abs(point.y) <= 0.75;
    ctx.fillStyle = detection ? 'rgba(71,85,105,.88)' : 'rgba(71,85,105,.46)';
    ctx.beginPath(); ctx.arc(screen.x, screen.y, detection ? 1.85 : 1.25, 0, Math.PI * 2); ctx.fill();
  });
  ctx.restore();
  if (!lidarLive) {
    ctx.fillStyle = '#9a6700';
    ctx.font = '600 14px Segoe UI';
    ctx.fillText(status.lidar_fresh ? '雷达本帧无有效回波' : '雷达数据已过期', 18, 70);
  }

  const localPlan = status.local_plan || [];
  if (localPlan.length) {
    ctx.strokeStyle = '#2384a8';
    ctx.lineWidth = 4;
    ctx.setLineDash([9, 6]);
    ctx.beginPath();
    localPlan.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
    ctx.setLineDash([]);
  }

  const route = activeRoute();
  if (route.length) {
    ctx.strokeStyle = '#167a67';
    ctx.lineWidth = 5;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    route.forEach((point, index) => {
      const screen = toScreen(point);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
    route.forEach((point, index) => {
      if (index === 0) return;
      const screen = toScreen(point);
      ctx.fillStyle = index === route.length - 1 ? '#c94343' : '#ffffff';
      ctx.strokeStyle = index === route.length - 1 ? '#c94343' : '#167a67';
      ctx.lineWidth = 3;
      ctx.beginPath(); ctx.arc(screen.x, screen.y, index === route.length - 1 ? 9 : 5, 0, Math.PI * 2);
      ctx.fill(); ctx.stroke();
    });
  } else if (status.target && status.target_active) {
    const target = toScreen(status.target);
    ctx.strokeStyle = '#c94343';
    ctx.lineWidth = 4;
    ctx.beginPath(); ctx.arc(target.x, target.y, 13, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#c94343';
    ctx.font = '700 22px Segoe UI';
    ctx.fillText('目标', target.x + 20, target.y + 7);
  }

  if (status.position) {
    ctx.save();
    ctx.translate(anchor.x, anchor.y);
    const halfWidth = 0.20 * scale;
    const front = 0;
    const rear = 0.62 * scale;
    ctx.fillStyle = '#ffffff';
    ctx.strokeStyle = '#167a67';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.roundRect(-halfWidth, -front, halfWidth * 2, rear, 6);
    ctx.fill(); ctx.stroke();
    ctx.fillStyle = '#167a67';
    ctx.beginPath(); ctx.moveTo(0, -8); ctx.lineTo(-6, 3);
    ctx.lineTo(6, 3); ctx.closePath(); ctx.fill();
    ctx.restore();
  }
}

function updateRouteControls() {
  ui.drawRoute.setAttribute('aria-pressed', String(drawMode));
  ui.drawRoute.textContent = drawMode ? '完成绘制' : '绘制路线';
  ui.canvas.classList.toggle('drawing', drawMode);
  ui.undoRoute.disabled = draftRoute.length < 2;
  ui.clearRoute.disabled = draftRoute.length === 0;
  ui.runRoute.disabled = !status.connected || draftRoute.length < 2;
}

function pointerToWorld(event) {
  const rect = ui.canvas.getBoundingClientRect();
  const px = (event.clientX - rect.left) * ui.canvas.width / rect.width;
  const py = (event.clientY - rect.top) * ui.canvas.height / rect.height;
  const view = routeView || viewGeometry();
  const bounds = { xMin: -0.72 * view.zoom, xMax: 1.78 * view.zoom,
    yMin: -2.5 * view.zoom, yMax: 2.5 * view.zoom };
  const pad = { left: 36, right: 20, top: 24, bottom: 24 };
  const scale = Math.min(
    (ui.canvas.width - pad.left - pad.right) / (bounds.yMax - bounds.yMin),
    (ui.canvas.height - pad.top - pad.bottom) / (bounds.xMax - bounds.xMin));
  const usedW = (bounds.yMax - bounds.yMin) * scale;
  const usedH = (bounds.xMax - bounds.xMin) * scale;
  const offsetX = pad.left + (ui.canvas.width - pad.left - pad.right - usedW) / 2;
  const offsetY = pad.top + (ui.canvas.height - pad.top - pad.bottom - usedH) / 2;
  const yaw = (status.position?.yaw || 0) * Math.PI / 180;
  const forward = bounds.xMax - (py - offsetY) / scale;
  const left = bounds.yMax - (px - offsetX) / scale;
  return {
    x: Math.round((view.x + Math.cos(yaw) * forward - Math.sin(yaw) * left) * 100) / 100,
    y: Math.round((view.y + Math.sin(yaw) * forward + Math.cos(yaw) * left) * 100) / 100,
  };
}

function appendRoutePoint(point, force = false) {
  const previous = draftRoute[draftRoute.length - 1];
  if (!previous || force || Math.hypot(point.x - previous.x, point.y - previous.y) >= 0.15) {
    draftRoute.push(point);
    if (draftRoute.length >= 160) {
      draftRoute = draftRoute.slice(0, 160);
      drawMode = false;
      pointerDrawing = false;
      routeView = null;
      showToast('路线已达到 160 个采样点', true);
    }
    updateRouteControls();
    drawMap();
  }
}

function render(next) {
  status = next;
  ui.connection.classList.toggle('online', next.connected);
  ui.connection.querySelector('strong').textContent = next.connected ? 'Odin 已连接' : 'Odin 未连接';
  ui.frame.textContent = next.frame || 'odom';
  ui.position.textContent = next.position ? `X ${next.position.x.toFixed(2)} · Y ${next.position.y.toFixed(2)}` : '--';
  ui.distance.textContent = next.distance == null ? '--' : `${next.distance.toFixed(2)} m`;
  ui.obstacle.textContent = next.obstacle == null ? '--' : (next.obstacle ? '前方有障碍' : '通道正常');
  ui.velocity.textContent = next.velocity?.fresh ? `X ${next.velocity.x.toFixed(2)} · Y ${next.velocity.y.toFixed(2)}` : '--';
  const routeCount = next.route?.length || 0;
  ui.targetState.textContent = next.navigation_state?.label || (next.target_active
    ? (next.obstacle ? '前方有障碍' : (next.planner_active ? '实时规划中' : '等待可行路径'))
    : '未设置目标');
  ui.event.textContent = next.event || '等待状态';
  ui.form.querySelector('.primary').disabled = !next.connected;
  updateRouteControls();
  drawMap();
}

async function refresh() {
  try {
    const next = await request('/api/status');
    lastStatusReceivedAt = Date.now();
    render(next);
  } catch (error) {
    const retainLidar = Date.now() - lastStatusReceivedAt < 2000;
    render({ ...status, connected: false, lidar_fresh: false,
      lidar_points_current: false,
      lidar_points: retainLidar ? (status.lidar_points || []) : [], velocity: {} });
  }
}

ui.form.addEventListener('submit', (event) => {
  event.preventDefault();
  const goal = { x: Number(ui.x.value), y: Number(ui.y.value) };
  if (!Number.isFinite(goal.x) || !Number.isFinite(goal.y)) return;
  pendingAction = { type: 'goal', payload: goal };
  ui.confirmLabel.textContent = '确认目标';
  ui.confirmCoordinates.textContent = `X ${goal.x.toFixed(2)} · Y ${goal.y.toFixed(2)}`;
  ui.confirmGoal.textContent = '确认前往';
  ui.dialog.showModal();
});

ui.drawRoute.addEventListener('click', () => {
  drawMode = !drawMode;
  routeView = drawMode ? {
    x: status.position?.x || 0,
    y: status.position?.y || 0,
    zoom: viewZoom,
  } : null;
  if (drawMode && !draftRoute.length && status.position) {
    draftRoute.push({ x: status.position.x, y: status.position.y });
  }
  updateRouteControls();
  drawMap();
});

ui.canvas.addEventListener('pointerdown', (event) => {
  if (!drawMode) return;
  pointerDrawing = true;
  ui.canvas.setPointerCapture(event.pointerId);
  appendRoutePoint(pointerToWorld(event), true);
});

ui.canvas.addEventListener('pointermove', (event) => {
  if (drawMode && pointerDrawing) appendRoutePoint(pointerToWorld(event));
});

ui.canvas.addEventListener('pointerup', (event) => {
  if (!drawMode || !pointerDrawing) return;
  pointerDrawing = false;
  appendRoutePoint(pointerToWorld(event));
  ui.canvas.releasePointerCapture(event.pointerId);
});

function finishPointer(event) {
  if (!pointerDrawing) return;
  pointerDrawing = false;
  if (ui.canvas.hasPointerCapture(event.pointerId)) ui.canvas.releasePointerCapture(event.pointerId);
}

ui.canvas.addEventListener('pointercancel', finishPointer);
ui.canvas.addEventListener('lostpointercapture', () => { pointerDrawing = false; });

ui.undoRoute.addEventListener('click', () => {
  if (draftRoute.length > 1) draftRoute.pop();
  updateRouteControls();
  drawMap();
});

ui.clearRoute.addEventListener('click', () => {
  draftRoute = drawMode && status.position
    ? [{ x: status.position.x, y: status.position.y }]
    : [];
  updateRouteControls();
  drawMap();
});

ui.zoomOut.addEventListener('click', () => {
  viewZoom = Math.min(4, viewZoom + 0.5);
  if (routeView) routeView.zoom = viewZoom;
  drawMap();
});

ui.zoomIn.addEventListener('click', () => {
  viewZoom = Math.max(0.5, viewZoom - 0.5);
  if (routeView) routeView.zoom = viewZoom;
  drawMap();
});

ui.runRoute.addEventListener('click', () => {
  const plannedRoute = simplifyRoute(draftRoute);
  const end = plannedRoute[plannedRoute.length - 1];
  pendingAction = { type: 'route', payload: { points: plannedRoute } };
  ui.confirmLabel.textContent = '确认路线';
  ui.confirmCoordinates.textContent = `${Math.max(1, plannedRoute.length - 1)} 个路点 · ${routeLength(plannedRoute).toFixed(1)} m · 终点 ${end.x.toFixed(1)}, ${end.y.toFixed(1)}`;
  ui.confirmGoal.textContent = '确认执行路线';
  ui.dialog.showModal();
});

ui.cancelConfirm.addEventListener('click', () => ui.dialog.close());
ui.confirmGoal.addEventListener('click', async () => {
  if (!pendingAction) return;
  ui.confirmGoal.disabled = true;
  try {
    const path = pendingAction.type === 'route' ? '/api/route' : '/api/goal';
    const result = await request(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(pendingAction.payload),
    });
    if (pendingAction.type === 'route') {
      drawMode = false;
      routeView = null;
      draftRoute = [];
    }
    ui.dialog.close();
    showToast(result.message);
    await refresh();
  } catch (error) {
    showToast(error.message, true);
  } finally {
    ui.confirmGoal.disabled = false;
  }
});

ui.stop.addEventListener('click', async () => {
  ui.stop.disabled = true;
  try {
    const result = await request('/api/stop', { method: 'POST', headers: { 'Content-Length': '0' } });
    showToast(result.message);
    await refresh();
  } catch (error) {
    showToast(error.message, true);
  } finally {
    ui.stop.disabled = false;
  }
});

window.addEventListener('resize', drawMap);
refresh();
setInterval(refresh, 700);
