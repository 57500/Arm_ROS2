// 界面逻辑：连接桥接节点、更新显示、把手势翻译成 IK / 执行请求。
// 这里不计算任何运动学，也不发速度指令。

import { RobotView } from './robot_view.js';
import { JointPlot } from './plot.js';

let lastTableAt = 0;
let lastPlotAt = 0;

const state = {
  ws: null,
  connected: false,
  jointNames: [],
  q: [],
  dq: [],
  ref: [],
  err: [],
  urdfLoaded: false,
  previewQ: null,
  ikSeq: 0,
  lastIkApplied: 0,
  executing: false,
  meshPackage: 'if_arm_lite',
  avoidCollisions: true,
};

const el = (id) => document.getElementById(id);

function showFatal(text) {
  const box = el('fatal');
  if (box) {
    box.textContent = String(text);
    box.classList.add('show');
  }
  log('error', String(text));
}

function setHint(text) {
  const hint = el('viewportHint');
  if (!hint) return;
  if (!text) {
    hint.classList.add('hidden');
  } else {
    hint.classList.remove('hidden');
    hint.textContent = text;
  }
}

window.addEventListener('error', (e) => showFatal('JS 错误: ' + e.message));
window.addEventListener('unhandledrejection', (e) => showFatal('Promise 异常: ' + e.reason));

let view;
try {
  view = new RobotView(el('viewport'), { tipLink: 'link6' });
} catch (err) {
  showFatal('初始化 3D 视图失败: ' + err.message);
  view = {
    jointNames: [],
    setJoints() {},
    setTargetJoints() {},
    setMode() {},
    snapToEE() {},
    resetView() {},
    loadURDF() {},
    getTargetPose() { return { pos: [0, 0, 0], quat: [0, 0, 0, 1] }; },
  };
}
let plot = null;

// ---------------- WebSocket ----------------
async function connect() {
  let wsPort = 8001;
  try {
    const cfg = await (await fetch('/config.json', { cache: 'no-store' })).json();
    wsPort = cfg.ws_port || wsPort;
  } catch (err) {
    log('warn', '读不到 /config.json，使用默认端口 8001');
  }

  const url = `ws://${location.hostname}:${wsPort}`;
  const ws = new WebSocket(url);
  state.ws = ws;
  log('info', `连接 ${url}`);

  ws.onopen = () => {
    state.connected = true;
    setConn(true);
    setHint('已连接，等待模型…');
    send({ type: 'set_ik', avoid_collisions: state.avoidCollisions });
  };
  ws.onclose = () => {
    state.connected = false;
    setConn(false);
    setHint('未连接桥接节点（3 秒后重连）');
    log('warn', '连接断开，3 秒后重连');
    setTimeout(connect, 3000);
  };
  ws.onerror = () => log('error', 'WebSocket 错误');
  ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
}

function send(payload) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(payload));
    return true;
  }
  log('error', '未连接，命令未发送');
  return false;
}

function handleMessage(msg) {
  switch (msg.type) {
    case 'hello':
      state.jointNames = msg.joint_names || [];
      view.jointNames = state.jointNames;
      if (msg.tip_link) view.tipLink = msg.tip_link;
      state.meshPackage = msg.mesh_package || 'if_arm_lite';
      state.q = msg.q || state.jointNames.map(() => 0);
      initTables();
      if (msg.urdf) loadURDF(msg.urdf);
      else setHint('已连接，等待 URDF…');
      break;

    case 'urdf':
      loadURDF(msg.urdf);
      break;

    case 'state':
      if (msg.jtc === false && !state.jtcWarned) {
        state.jtcWarned = true;
        log('warn', '未收到 /manipulator_controller/state，参考/误差暂时为 0');
      }
      state.q = msg.q || state.q;
      state.dq = msg.dq || state.dq;
      state.ref = msg.ref || state.ref;
      state.err = msg.err || state.err;
      if (!state.previewQ) {
        view.setTargetJoints(state.q);
        syncSliders(state.q);
      }
      {
        const now = performance.now();
        if (now - lastTableAt > 100) {
          lastTableAt = now;
          updateJointTable(state.q, state.dq);
          updateLegend(state.q);
        }
        if (plot && now - lastPlotAt > 50) {
          lastPlotAt = now;
          plot.push(msg.t || Date.now() / 1000, {
            pos: state.q, vel: state.dq, ref: state.ref, err: state.err,
          });
        }
      }
      break;

    case 'status':
      renderStatus(msg);
      break;

    case 'ik':
      if (msg.seq < state.lastIkApplied) return;
      state.lastIkApplied = msg.seq;
      if (msg.ok) {
        state.previewQ = msg.q;
        view.setJoints(msg.q);
        el('btnExecute').disabled = state.executing;
        el('ikInfo').textContent = 'IK：成功';
        el('ikInfo').style.color = 'var(--ok)';
      } else {
        el('ikInfo').textContent = `IK：${msg.error}`;
        el('ikInfo').style.color = 'var(--bad)';
      }
      break;

    case 'execute':
      state.executing = false;
      el('btnExecute').disabled = false;
      log(msg.ok ? 'info' : 'error', msg.ok ? '执行完成' : `执行失败：${msg.error}`);
      if (msg.ok) {
        state.previewQ = null;
        view.snapToEE();
      }
      break;

    case 'estop':
      log('warn', `急停：${msg.detail}`);
      state.previewQ = null;
      break;

    case 'motor_config':
      renderMotorConfig(msg);
      break;

    case 'motor_result':
      log(msg.ok ? 'info' : 'error', `电机 ${msg.id} ${msg.action}：${msg.detail}`);
      if (msg.ok && msg.action !== 'read') el('dbgRead').click();
      break;

    case 'log':
      log(msg.level || 'info', msg.msg);
      break;

    default:
      break;
  }
}

// ---------------- 渲染 ----------------
function setConn(ok) {
  const badge = el('conn');
  badge.textContent = ok ? '已连接' : '未连接';
  badge.className = `badge ${ok ? 'ok' : 'bad'}`;
}

function loadURDF(xml) {
  if (state.urdfLoaded || !xml) return;
  state.urdfLoaded = true;
  log('info', `收到 URDF（${xml.length} 字节），开始加载网格`);
  try {
    view.loadURDF(xml, state.meshPackage);
    view.onGizmoChange = scheduleIK;
    view.setJoints(state.q);
    applyJointLimits(xml);
    syncSliders(state.q);
    setHint('');
  } catch (err) {
    showFatal('加载 URDF 失败: ' + err.message);
  }
}

function initTables() {
  const tbody = el('jointTable').querySelector('tbody');
  tbody.innerHTML = '';
  state.jointNames.forEach((name) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${name}</td><td class="pos">0.0</td><td class="vel">0.0</td>`;
    tbody.appendChild(tr);
  });

  buildSliders();
  buildJointToggles();

  if (!plot) {
    try {
      plot = new JointPlot(el('plot'), state.jointNames);
    } catch (err) {
      showFatal('波形初始化失败: ' + err.message);
    }
  }
}

function updateJointTable(q, dq) {
  el('jointTable').querySelectorAll('tbody tr').forEach((row, i) => {
    row.querySelector('.pos').textContent = (((q[i] || 0) * 180) / Math.PI).toFixed(1);
    row.querySelector('.vel').textContent = (((dq?.[i] || 0) * 180) / Math.PI).toFixed(1);
  });
}

function renderStatus(msg) {
  el('controllers').innerHTML = (msg.controllers || [])
    .map((c) => `<div class="item"><span>${c.name}</span><span>${c.state}</span></div>`)
    .join('');
  el('hardware').innerHTML = (msg.hardware || [])
    .map((h) => `<div class="item"><span>${h.name}</span><span>${h.state}</span></div>`)
    .join('');
}

function log(level, text) {
  const box = el('log');
  if (!box) return;
  const line = document.createElement('div');
  line.className = level;
  line.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  while (box.childElementCount > 300) box.removeChild(box.firstChild);
}

// ---------------- 关节滑块 ----------------
let sliderInputs = [];

function buildSliders() {
  const box = el('jointSliders');
  if (!box) return;
  box.innerHTML = '';
  sliderInputs = [];

  state.jointNames.forEach((name, i) => {
    const row = document.createElement('div');
    row.className = 'slider-row';

    const label = document.createElement('span');
    label.className = 'name';
    label.textContent = name;

    const input = document.createElement('input');
    input.type = 'range';
    input.min = '-180';
    input.max = '180';
    input.step = '0.5';
    input.value = String(((state.q[i] || 0) * 180) / Math.PI);

    const val = document.createElement('span');
    val.className = 'val';
    val.textContent = Number(input.value).toFixed(1);

    input.oninput = () => {
      const deg = Number(input.value);
      val.textContent = deg.toFixed(1);
      state.previewQ = (state.previewQ || state.q.slice()).slice();
      state.previewQ[i] = (deg * Math.PI) / 180;
      view.setJoints(state.previewQ);
      el('btnExecute').disabled = state.executing;
      el('ikInfo').textContent = '滑块预览（点执行生效）';
      el('ikInfo').style.color = 'var(--accent)';
    };

    row.appendChild(label);
    row.appendChild(input);
    row.appendChild(val);
    box.appendChild(row);
    sliderInputs.push(input);
  });
}

function syncSliders(q) {
  sliderInputs.forEach((input, i) => {
    const deg = ((q[i] || 0) * 180) / Math.PI;
    input.value = String(deg);
    const val = input.parentElement.querySelector('.val');
    if (val) val.textContent = deg.toFixed(1);
  });
}

function applyJointLimits(xml) {
  try {
    const doc = new DOMParser().parseFromString(xml, 'text/xml');
    doc.querySelectorAll('joint').forEach((joint) => {
      const name = joint.getAttribute('name');
      const limit = joint.querySelector('limit');
      const idx = state.jointNames.indexOf(name);
      if (idx < 0 || !limit) return;
      const lower = parseFloat(limit.getAttribute('lower'));
      const upper = parseFloat(limit.getAttribute('upper'));
      const input = sliderInputs[idx];
      if (!input) return;
      if (Number.isFinite(lower)) input.min = String((lower * 180) / Math.PI);
      if (Number.isFinite(upper)) input.max = String((upper * 180) / Math.PI);
    });
  } catch (err) {
    log('warn', '解析关节限位失败: ' + err.message);
  }
}

// ---------------- 波形：关节选择 + 当前值 ----------------
function buildJointToggles() {
  const box = el('jointToggles');
  if (!box) return;
  const colors = plot ? plot.colors() : ['#4da3ff'];
  box.innerHTML = state.jointNames
    .map((name, i) => {
      const color = colors[i % colors.length];
      return `<span class="on" data-joint="${i}"><i style="background:${color}"></i>${name}</span>`;
    })
    .join('');

  box.querySelectorAll('span').forEach((chip) => {
    chip.onclick = () => {
      const i = Number(chip.dataset.joint);
      if (plot) plot.toggleJoint(i);
      chip.classList.toggle('on');
    };
  });
}

let legendStamp = 0;
function updateLegend(q) {
  // 关节选择行只显示名字和开关状态，具体数值在"状态"页的表格里
  return;
}

// ---------------- 分页 ----------------
function bindTabs() {
  el('tabs').querySelectorAll('button').forEach((btn) => {
    btn.onclick = () => {
      el('tabs').querySelectorAll('button').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      document.querySelectorAll('.pane').forEach((p) => p.classList.remove('active'));
      const pane = el(`tab-${btn.dataset.tab}`);
      if (pane) pane.classList.add('active');
      if (plot) setTimeout(() => plot.resize(), 0);
    };
  });
}

// ---------------- 波形高度拖动 ----------------
function bindSplitter() {
  const app = el('app');
  const split = el('splitter');
  let dragging = false;

  split.addEventListener('pointerdown', (e) => {
    dragging = true;
    split.setPointerCapture(e.pointerId);
    document.body.style.userSelect = 'none';
  });
  split.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    const rect = app.getBoundingClientRect();
    let h = rect.bottom - e.clientY;
    h = Math.max(90, Math.min(rect.height - 220, h));
    app.style.gridTemplateRows = `1fr 6px ${h}px`;
    if (plot) plot.resize();
  });
  split.addEventListener('pointerup', (e) => {
    dragging = false;
    document.body.style.userSelect = '';
    split.releasePointerCapture(e.pointerId);
  });
}

// ---------------- IK 节流（只保留最新一次） ----------------
let ikTimer = null;

function scheduleIK() {
  if (ikTimer) return;
  ikTimer = setTimeout(() => {
    ikTimer = null;
    requestIK();
  }, 33);
}

function requestIK() {
  const pose = view.getTargetPose();
  const seq = ++state.ikSeq;
  send({
    type: 'ik',
    seq,
    pos: pose.pos,
    quat: pose.quat,
    seed: state.previewQ || state.q,
  });
}

// ---------------- 电机调试 ----------------
function renderMotorConfig(msg) {
  const box = el('dbgConfig');
  if (!box) return;
  const f = msg.fields || {};
  const lines = Object.keys(f).map((k) => `${k.padEnd(14)} ${f[k]}`);
  box.textContent = `电机 ${msg.id}\n` + lines.join('\n') + `\n\n原始: ${msg.raw || ''}`;
}

function bindDebug() {
  el('dbgRead').onclick = () => {
    send({ type: 'motor_read', id: Number(el('dbgMotor').value) });
  };

  el('dbgAckApply').onclick = () => {
    const id = Number(el('dbgMotor').value);
    const value = Number(el('dbgAck').value);
    if (!confirm(`把电机 ${id} 的控制命令应答方式改为 ${el('dbgAck').selectedOptions[0].text} 并写入 flash？`)) return;
    send({ type: 'motor_write', id, field: 'ack', value, save: true });
  };

  el('dbgBitrateApply').onclick = () => {
    const id = Number(el('dbgMotor').value);
    const value = Number(el('dbgBitrate').value);
    if (!confirm(`把电机 ${id} 的 CAN 波特率改成 ${el('dbgBitrate').selectedOptions[0].text}？\n改完必须重新上电，并把本机 can0 也改到相同速率，否则会失联。`)) return;
    send({ type: 'motor_write', id, field: 'bitrate', value, save: true });
  };

  el('dbgZero').onclick = () => {
    const id = Number(el('dbgMotor').value);
    if (!confirm(`把电机 ${id} 当前位置设为零点（单圈零点）？`)) return;
    send({ type: 'motor_zero', id });
  };
}

// ---------------- 交互 ----------------
function bindUI() {
  bindTabs();
  bindSplitter();
  bindDebug();

  el('modeTranslate').onclick = () => {
    view.setMode('translate');
    el('modeTranslate').classList.add('active');
    el('modeRotate').classList.remove('active');
  };

  el('modeRotate').onclick = () => {
    view.setMode('rotate');
    el('modeRotate').classList.add('active');
    el('modeTranslate').classList.remove('active');
  };

  el('ikCollision').onchange = () => {
    state.avoidCollisions = el('ikCollision').checked;
    const badge = el('ikMode');
    badge.textContent = state.avoidCollisions ? '开' : '关';
    badge.className = `badge ${state.avoidCollisions ? 'ok' : 'bad'}`;
    send({ type: 'set_ik', avoid_collisions: state.avoidCollisions });
  };

  el('btnSnap').onclick = () => {
    view.snapToEE();
    state.previewQ = null;
    el('btnExecute').disabled = true;
  };

  el('btnCancelPreview').onclick = () => {
    state.previewQ = null;
    view.setJoints(state.q);
    view.snapToEE();
    el('btnExecute').disabled = true;
    el('ikInfo').textContent = 'IK：—';
    el('ikInfo').style.color = 'var(--muted)';
  };

  el('btnView').onclick = () => view.resetView();

  el('btnSyncSliders').onclick = () => {
    state.previewQ = null;
    view.setJoints(state.q);
    syncSliders(state.q);
    el('btnExecute').disabled = true;
    el('ikInfo').textContent = 'IK：—';
    el('ikInfo').style.color = 'var(--muted)';
  };

  const vel = el('velScale');
  const acc = el('accScale');
  vel.oninput = () => { el('velScaleVal').textContent = Number(vel.value).toFixed(2); };
  acc.oninput = () => { el('accScaleVal').textContent = Number(acc.value).toFixed(2); };

  el('btnExecute').onclick = () => {
    if (!state.previewQ) {
      log('warn', '还没有预览结果');
      return;
    }
    state.executing = true;
    el('btnExecute').disabled = true;
    send({
      type: 'execute',
      q: state.previewQ,
      vel_scale: Number(vel.value),
      acc_scale: Number(acc.value),
    });
  };

  el('btnHome').onclick = () => {
    state.executing = true;
    send({
      type: 'execute',
      q: state.jointNames.map(() => 0),
      vel_scale: Number(vel.value),
      acc_scale: Number(acc.value),
    });
  };

  el('btnEstop').onclick = () => {
    if (!confirm('急停会停控制器并把硬件切到 inactive，确认？')) return;
    send({ type: 'estop' });
  };

  // 波形：四个信号可任意组合
  const sigBind = (id, key) => {
    const cb = el(id);
    if (!cb) return;
    cb.onchange = () => { if (plot) plot.setSignal(key, cb.checked); };
  };
  sigBind('sigPos', 'pos');
  sigBind('sigVel', 'vel');
  sigBind('sigRef', 'ref');
  sigBind('sigErr', 'err');
  el('plotZoomInY').onclick = () => plot && plot.zoomY(1 / 1.3);
  el('plotZoomOutY').onclick = () => plot && plot.zoomY(1.3);
  el('plotReset').onclick = () => plot && plot.resetAxes();
}

bindUI();
connect();



