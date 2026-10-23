// 多信号波形：实际/期望的位置与速度、位置/速度误差，可任意组合。
// 交互：滚轮缩放 Y，左键拖动平移 X，悬停显示最近的曲线信息。

export const COLORS = ['#4da3ff', '#3fb950', '#d29922', '#f85149', '#a371f7', '#39c5cf'];

// 每个关节一个基准色相，6 个信号在同色相下错开亮度/饱和度，
// 36 条曲线都是实线且颜色互不相同，同时还能按"色系"认出是同一关节。
const JOINT_HUE = [205, 145, 45, 352, 275, 182];

function colorFor(signalIndex, jointIndex) {
  const hue = (JOINT_HUE[jointIndex % JOINT_HUE.length] + (signalIndex - 2) * 26 + 360) % 360;
  const sat = 70 + (signalIndex % 3) * 10;
  const light = 45 + signalIndex * 5.5;
  return `hsl(${hue}, ${sat}%, ${light}%)`;
}

export const SIGNALS = [
  { key: 'pos', label: '实际位置', unit: '°', width: 1.7 },
  { key: 'vel', label: '实际速度', unit: '°/s', width: 1.6 },
  { key: 'ref', label: '期望位置', unit: '°', width: 1.6 },
  { key: 'ref_vel', label: '期望速度', unit: '°/s', width: 1.6 },
  { key: 'err', label: '位置误差', unit: '°', width: 1.5 },
  { key: 'err_vel', label: '速度误差', unit: '°/s', width: 1.5 },
];

const WINDOW_SEC = 30;

export class JointPlot {
  constructor(el, jointNames, tipEl) {
    this.el = el;
    this.tipEl = tipEl || null;
    this.jointNames = jointNames;
    this.active = { pos: true, vel: true, ref: false, ref_vel: false, err: false, err_vel: false };
    this.visible = jointNames.map(() => true);
    this.yManual = null;
    this.panning = false;
    this.userPanned = false;
    this.uplot = null;

    this.history = { t: [] };
    SIGNALS.forEach((sig) => {
      this.history[sig.key] = jointNames.map(() => []);
    });

    this.bindInteractions();
    this.rebuild();
    new ResizeObserver(() => this.resize()).observe(el);
  }

  seriesList() {
    const out = [];
    SIGNALS.forEach((sig, si) => {
      if (!this.active[sig.key]) return;
      this.jointNames.forEach((name, i) => {
        if (!this.visible[i]) return;
        out.push({
          signal: sig.key,
          unit: sig.unit,
          joint: i,
          label: `${name} ${sig.label}`,
          stroke: colorFor(si, i),
          width: sig.width,
          // 位置类(°)走左轴，速度类(°/s)走右轴，量级差 10 倍也不会互相压扁
          scale: sig.unit === '°/s' ? 'y2' : 'y',
          points: { show: false },
        });
      });
    });
    return out;
  }

  options() {
    const series = this.seriesList();
    const used = new Set(series.map((s) => s.scale));
    const axes = [
      { stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' } },
    ];
    if (used.has('y')) {
      axes.push({
        scale: 'y', side: 3, size: 64, label: '位置 (°)', labelSize: 18,
        stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' },
      });
    }
    if (used.has('y2')) {
      axes.push({
        scale: 'y2', side: 3, size: 64, label: '速度 (°/s)', labelSize: 18,
        stroke: '#7d8b9e', grid: { show: false }, ticks: { stroke: '#2b323c' },
      });
    }
    return {
      width: this.el.clientWidth || 600,
      height: this.el.clientHeight || 160,
      padding: [6, 10, 0, 0],
      cursor: { drag: { x: false, y: false } },
      scales: { x: { time: true }, y: {}, y2: {} },
      legend: { show: false },
      axes,
      series: [{ label: '时间' }].concat(series),
    };
  }

  buildData() {
    const data = [this.history.t];
    this.seriesList().forEach((s) => {
      data.push(this.history[s.signal][s.joint]);
    });
    return data;
  }

  rebuild() {
    if (this.uplot) this.uplot.destroy();
    this.uplot = new uPlot(this.options(), this.buildData(), this.el);
    this.applyYManual();
    if (!this.userPanned) this.applyXWindow();
  }

  applyYManual() {
    if (!this.uplot || !this.yManual) return;
    Object.keys(this.yManual).forEach((key) => {
      this.uplot.setScale(key, this.yManual[key]);
    });
  }

  /** 滚动时间窗：显示最近 WINDOW_SEC 秒，用户平移后暂停跟随 */
  applyXWindow() {
    if (!this.uplot || !this.history.t.length) return;
    const t0 = this.history.t[0];
    const t1 = this.history.t[this.history.t.length - 1];
    let min = Math.max(t0, t1 - WINDOW_SEC);
    if (t1 - min < 1e-3) min = t1 - 1;
    this.uplot.setScale('x', { min, max: t1 });
  }

  resize() {
    const width = this.el.clientWidth;
    const height = this.el.clientHeight;
    if (this.uplot && width > 0 && height > 0) {
      this.uplot.setSize({ width, height });
    }
  }

  push(t, values) {
    this.history.t.push(t);
    SIGNALS.forEach((sig) => {
      this.history[sig.key].forEach((arr, i) => {
        const v = values[sig.key];
        arr.push(v ? v[i] : null);
      });
    });

    const limit = t - WINDOW_SEC;
    while (this.history.t.length && this.history.t[0] < limit) {
      this.history.t.shift();
      SIGNALS.forEach((sig) => {
        this.history[sig.key].forEach((arr) => arr.shift());
      });
    }
    if (this.uplot) {
      this.uplot.setData(this.buildData());
      if (!this.userPanned) this.applyXWindow();
      this.applyYManual();
    }
  }

  setSignal(key, on) {
    this.active[key] = on;
    this.rebuild();
  }

  toggleJoint(index) {
    this.visible[index] = !this.visible[index];
    this.rebuild();
  }

  resetX() {
    this.userPanned = false;
    this.applyXWindow();
  }

  resetAxes() {
    this.yManual = null;
    if (this.uplot) {
      this.uplot.setScale('y', { auto: true });
      this.uplot.setScale('y2', { auto: true });
      this.uplot.setData(this.buildData());
    }
    this.resetX();
  }

  zoomY(factor) {
    if (!this.uplot) return;
    const manual = {};
    ['y', 'y2'].forEach((key) => {
      const range = this.uplot.scales[key];
      if (!range || !Number.isFinite(range.min) || !Number.isFinite(range.max)) return;
      const center = (range.min + range.max) / 2;
      const half = Math.max(1e-3, ((range.max - range.min) / 2) * factor);
      manual[key] = { min: center - half, max: center + half };
    });
    if (!Object.keys(manual).length) return;
    this.yManual = manual;
    this.applyYManual();
  }

  // ---- 悬停提示 ----
  showTip(clientX, clientY, info) {
    if (!this.tipEl) return;
    const host = this.tipEl.parentElement.getBoundingClientRect();
    this.tipEl.innerHTML = `<i style="background:${info.stroke}"></i>`
      + `${info.label}：${info.value.toFixed(2)} ${info.unit}`;
    this.tipEl.style.left = `${clientX - host.left + 12}px`;
    this.tipEl.style.top = `${clientY - host.top + 12}px`;
    this.tipEl.classList.add('show');
  }

  hideTip() {
    if (this.tipEl) this.tipEl.classList.remove('show');
  }

  updateHover(event) {
    if (!this.uplot || this.panning) {
      this.hideTip();
      return;
    }
    const idx = this.uplot.cursor.idx;
    if (idx == null) {
      this.hideTip();
      return;
    }
    const rect = this.el.getBoundingClientRect();
    const y = event.clientY - rect.top;

    let best = null;
    this.seriesList().forEach((s) => {
      const value = this.history[s.signal][s.joint][idx];
      if (value == null || !Number.isFinite(value)) return;
      const py = this.uplot.valToPos(value, s.scale);
      const dist = Math.abs(py - y);
      if (dist < 22 && (!best || dist < best.dist)) {
        best = { dist, label: s.label, value, unit: s.unit, stroke: s.stroke };
      }
    });

    if (best) this.showTip(event.clientX, event.clientY, best);
    else this.hideTip();
  }

  bindInteractions() {
    const el = this.el;

    el.addEventListener('wheel', (event) => {
      event.preventDefault();
      this.zoomY(event.deltaY > 0 ? 1.15 : 1 / 1.15);
    }, { passive: false });

    el.addEventListener('pointerdown', (event) => {
      if (event.button !== 0 || !this.uplot) return;
      const scale = this.uplot.scales.x;
      if (!Number.isFinite(scale.max)) return;
      this.panning = {
        x: event.clientX,
        min: scale.min,
        max: scale.max,
        width: el.clientWidth || 1,
      };
      this.userPanned = true;
      el.setPointerCapture(event.pointerId);
      el.style.cursor = 'grabbing';
      this.hideTip();
    });

    el.addEventListener('pointermove', (event) => {
      if (this.panning && this.uplot) {
        const dt = ((event.clientX - this.panning.x) / this.panning.width)
          * (this.panning.max - this.panning.min);
        this.uplot.setScale('x', {
          min: this.panning.min - dt,
          max: this.panning.max - dt,
        });
        return;
      }
      this.updateHover(event);
    });

    const stop = (event) => {
      if (!this.panning) return;
      this.panning = false;
      el.style.cursor = '';
      try { el.releasePointerCapture(event.pointerId); } catch (e) { /* ignore */ }
    };
    el.addEventListener('pointerup', stop);
    el.addEventListener('pointercancel', stop);
    el.addEventListener('pointerleave', () => this.hideTip());
  }

  colors() {
    return COLORS;
  }
}
