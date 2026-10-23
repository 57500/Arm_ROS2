// 多信号波形：位置 / 速度 / 参考 / 误差，可任意组合显示。
// 交互：滚轮缩放 Y，左键拖动平移 X，按钮重置。

export const COLORS = ['#4da3ff', '#3fb950', '#d29922', '#f85149', '#a371f7', '#39c5cf'];

export const SIGNALS = [
  { key: 'pos', label: '位置', width: 1.7, dash: null },
  { key: 'vel', label: '速度', width: 1.2, dash: [6, 4] },
  { key: 'ref', label: '参考', width: 1.2, dash: [2, 3] },
  { key: 'err', label: '误差', width: 1.0, dash: [10, 4] },
];

const WINDOW_SEC = 30;

export class JointPlot {
  constructor(el, jointNames) {
    this.el = el;
    this.jointNames = jointNames;
    this.active = { pos: true, vel: false, ref: false, err: false };
    this.visible = jointNames.map(() => true);
    this.yManual = null;
    this.panning = false;
    this.uplot = null;

    this.history = {
      t: [],
      pos: jointNames.map(() => []),
      vel: jointNames.map(() => []),
      ref: jointNames.map(() => []),
      err: jointNames.map(() => []),
    };

    this.bindInteractions();
    this.rebuild();
    new ResizeObserver(() => this.resize()).observe(el);
  }

  // ---- 序列定义 ----
  seriesList() {
    const out = [];
    SIGNALS.forEach((sig) => {
      if (!this.active[sig.key]) return;
      this.jointNames.forEach((name, i) => {
        if (!this.visible[i]) return;
        out.push({
          signal: sig.key,
          joint: i,
          label: `${name} ${sig.label}`,
          stroke: COLORS[i % COLORS.length],
          width: sig.width,
          dash: sig.dash || undefined,
          points: { show: false },
        });
      });
    });
    return out;
  }

  options() {
    return {
      width: this.el.clientWidth || 600,
      height: this.el.clientHeight || 160,
      padding: [6, 10, 0, 0],
      cursor: { drag: { x: false, y: false } },
      scales: { x: { time: true }, y: { auto: true } },
      legend: { show: false },
      axes: [
        { stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' } },
        { stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' }, size: 52 },
      ],
      series: [{ label: '时间' }].concat(this.seriesList()),
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
    let xRange = null;
    if (this.uplot) {
      xRange = { min: this.uplot.scales.x.min, max: this.uplot.scales.x.max };
      this.uplot.destroy();
    }
    this.uplot = new uPlot(this.options(), this.buildData(), this.el);
    if (xRange && Number.isFinite(xRange.min)) {
      this.uplot.setScale('x', xRange);
    } else {
      this.resetX();
    }
    if (this.yManual) {
      this.uplot.setScale('y', this.yManual);
    }
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
    ['pos', 'vel', 'ref', 'err'].forEach((key) => {
      this.history[key].forEach((arr, i) => {
        arr.push(values[key] ? values[key][i] : null);
      });
    });

    const limit = t - WINDOW_SEC;
    while (this.history.t.length && this.history.t[0] < limit) {
      this.history.t.shift();
      ['pos', 'vel', 'ref', 'err'].forEach((key) => {
        this.history[key].forEach((arr) => arr.shift());
      });
    }
    if (this.uplot) {
      this.uplot.setData(this.buildData());
      if (this.yManual) this.uplot.setScale('y', this.yManual);
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
    const t = this.history.t;
    if (!this.uplot) return;
    const now = t.length ? t[t.length - 1] : Date.now() / 1000;
    this.uplot.setScale('x', { min: now - WINDOW_SEC, max: now });
  }

  resetAxes() {
    this.yManual = null;
    if (this.uplot) this.uplot.setScale('y', { auto: true });
    this.resetX();
  }

  zoomY(factor) {
    if (!this.uplot) return;
    const range = this.uplot.scales.y || {};
    const min = range.min ?? -180;
    const max = range.max ?? 180;
    const center = (min + max) / 2;
    const half = Math.max(0.05, ((max - min) / 2) * factor);
    this.yManual = { min: center - half, max: center + half };
    this.uplot.setScale('y', this.yManual);
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
      el.setPointerCapture(event.pointerId);
      el.style.cursor = 'grabbing';
    });

    el.addEventListener('pointermove', (event) => {
      if (!this.panning || !this.uplot) return;
      const dt = ((event.clientX - this.panning.x) / this.panning.width)
        * (this.panning.max - this.panning.min);
      this.uplot.setScale('x', {
        min: this.panning.min - dt,
        max: this.panning.max - dt,
      });
    });

    const stop = (event) => {
      if (!this.panning) return;
      this.panning = false;
      el.style.cursor = '';
      try { el.releasePointerCapture(event.pointerId); } catch (e) { /* ignore */ }
    };
    el.addEventListener('pointerup', stop);
    el.addEventListener('pointercancel', stop);
  }

  colors() {
    return COLORS;
  }
}
