// 界面逻辑：连接桥接节点、更新显示、把手势翻译成 IK / 执行请求。
// 这里不计算任何运动学，也不发速度指令。

import { RobotView } from './robot_view.js';
import { JointPlot } from './plot.js';

const state = {
  ws: null,
  connected: false,
  jointNames: [],
  q: [],
  urdfLoaded: false,
  previewQ: null,
  ikSeq: 0,
  lastIkApplied: 0,
  executing: false,
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

// 3D 初始化失败时用空对象兜底，保证状态/波形/按钮仍然可用
let view;
try {
  view = new RobotView(el('viewport'), { tipLink: 'link6', jointNames: [] });
} catch (err) {
  showFatal('初始化 3D 视图失败: ' + err.message);
  view = {
    jointNames: [],
    setJoints() {},
    setMode() {},
    snapToEE() {},
    loadURDF() {},
    getTargetPose() {
      return { pos: [0, 0, 0], quat: [0, 0, 0, 1] };
    },
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
    log('info', 'WebSocket 已连接');
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
      if (msg.tip_link) {
        view.tipLink = msg.tip_link;
      }
      state.meshPackage = msg.mesh_package || 'if_arm_lite';
      state.q = msg.q || state.jointNames.map(() => 0);
      initTables();
      if (msg.urdf) {
        loadURDF(msg.urdf);
      } else {
        setHint('已连接，等待 URDF…');
      }
      break;

    case 'urdf':
      loadURDF(msg.urdf);
      break;

    case 'state':
      state.q = msg.q || state.q;
      updateJointTable(state.q, msg.dq);
      if (!state.previewQ) {
        view.setJoints(state.q);
      }
      if (plot) {
        plot.push(msg.t || Date.now() / 1000, state.q);
      }
      break;

    case 'status':
      renderStatus(msg);
      break;

    case 'ik':
      if (msg.seq < state.lastIkApplied) {
        return; // 旧请求的结果，丢掉
      }
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
  if (state.urdfLoaded || !xml) {
    return;
  }
  state.urdfLoaded = true;
  log('info', `收到 URDF（${xml.length} 字节），开始加载网格`);
  try {
    view.loadURDF(xml, state.meshPackage || 'if_arm_lite');
    view.onGizmoChange = scheduleIK;
    view.setJoints(state.q);
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
  const line = document.createElement('div');
  line.className = level;
  line.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
  while (box.childElementCount > 300) {
    box.removeChild(box.firstChild);
  }
}

// ---------------- IK 节流（只保留最新一次） ----------------
let ikTimer = null;

function scheduleIK() {
  if (ikTimer) {
    return;
  }
  ikTimer = setTimeout(() => {
    ikTimer = null;
    requestIK();
  }, 33); // 约 30 Hz
}

function requestIK() {
  const pose = view.getTargetPose();
  const seq = ++state.ikSeq;
  send({
    type: 'ik',
    seq,
    pos: pose.pos,
    quat: pose.quat,
    seed: state.previewQ || state.q, // 和 RViz 一样：用上一次的解作为下一次的种子
  });
}

// ---------------- 交互 ----------------
function bindUI() {
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

  const vel = el('velScale');
  const acc = el('accScale');
  vel.oninput = () => {
    el('velScaleVal').textContent = Number(vel.value).toFixed(2);
  };
  acc.oninput = () => {
    el('accScaleVal').textContent = Number(acc.value).toFixed(2);
  };

  el('btnExecute').onclick = () => {
    if (!state.previewQ) {
      log('warn', '还没有 IK 预览结果');
      return;
    }
    if (!confirm('确认执行当前预览位姿？')) {
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
    if (!confirm('确认回到全零位姿？')) {
      return;
    }
    state.executing = true;
    send({
      type: 'execute',
      q: state.jointNames.map(() => 0),
      vel_scale: Number(vel.value),
      acc_scale: Number(acc.value),
    });
  };

  el('btnEstop').onclick = () => {
    if (!confirm('急停会停控制器并把硬件切到 inactive，确认？')) {
      return;
    }
    send({ type: 'estop' });
  };
}

bindUI();
connect();

