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

let status = { position: null, target: null, route: [] };
let pendingAction = null;
let draftRoute = [];
let drawMode = false;
let pointerDrawing = false;
let routeView = null;
let routeRadius = 5;
let toastTimer = null;

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
  return draftRoute.length ? draftRoute : (status.route || []);
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
  if (drawMode && routeView) return routeView;
  const center = status.position || { x: 0, y: 0 };
  const points = [...activeRoute(), status.target].filter(Boolean);
  const maxOffset = points.length ? Math.max(...points.flatMap((point) => [
    Math.abs(point.x - center.x), Math.abs(point.y - center.y),
  ])) : 0;
  return { x: center.x, y: center.y, radius: Math.max(5, Math.ceil(maxOffset + 1)) };
}

function drawMap() {
  const canvas = ui.canvas;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const view = viewGeometry();
  const scale = Math.min(w, h) * 0.43 / view.radius;
  const toScreen = (point) => ({
    x: w / 2 + (point.x - view.x) * scale,
    y: h / 2 - (point.y - view.y) * scale,
  });

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#f8faf9';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#dce3e4';
  ctx.lineWidth = 1;
  const step = view.radius > 8 ? 2 : 1;
  for (let value = -view.radius; value <= view.radius; value += step) {
    const x = w / 2 + value * scale;
    const y = h / 2 - value * scale;
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  const origin = toScreen({ x: 0, y: 0 });
  ctx.strokeStyle = '#8e9aa2';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(origin.x, 0); ctx.lineTo(origin.x, h); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, origin.y); ctx.lineTo(w, origin.y); ctx.stroke();

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
  } else if (status.target) {
    const target = toScreen(status.target);
    ctx.strokeStyle = '#c94343';
    ctx.lineWidth = 4;
    ctx.beginPath(); ctx.arc(target.x, target.y, 13, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = '#c94343';
    ctx.font = '700 22px Segoe UI';
    ctx.fillText('目标', target.x + 20, target.y + 7);
  }

  if (status.position) {
    const position = toScreen(status.position);
    const angle = -(status.position.yaw || 0) * Math.PI / 180;
    ctx.save();
    ctx.translate(position.x, position.y);
    ctx.rotate(angle);
    ctx.fillStyle = '#167a67';
    ctx.beginPath(); ctx.moveTo(17, 0); ctx.lineTo(-11, -10); ctx.lineTo(-7, 0); ctx.lineTo(-11, 10); ctx.closePath(); ctx.fill();
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
  const scale = Math.min(ui.canvas.width, ui.canvas.height) * 0.43 / view.radius;
  return {
    x: Math.round((view.x + (px - ui.canvas.width / 2) / scale) * 100) / 100,
    y: Math.round((view.y - (py - ui.canvas.height / 2) / scale) * 100) / 100,
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
  ui.targetState.textContent = next.target_active && routeCount
    ? `路点 ${next.route_index + 1} / ${routeCount}`
    : (next.target_active ? '正在前往目标' : (next.target ? '目标已停止' : '未设置目标'));
  ui.event.textContent = next.event || '等待状态';
  ui.form.querySelector('.primary').disabled = !next.connected;
  updateRouteControls();
  drawMap();
}

async function refresh() {
  try {
    render(await request('/api/status'));
  } catch (error) {
    render({ connected: false, position: null, target: null, route: [], velocity: {} });
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
    radius: routeRadius,
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
  draftRoute = [];
  updateRouteControls();
  drawMap();
});

ui.zoomOut.addEventListener('click', () => {
  routeRadius = Math.min(20, routeRadius + 2);
  if (routeView) routeView.radius = routeRadius;
  drawMap();
});

ui.zoomIn.addEventListener('click', () => {
  routeRadius = Math.max(3, routeRadius - 2);
  if (routeView) routeView.radius = routeRadius;
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
