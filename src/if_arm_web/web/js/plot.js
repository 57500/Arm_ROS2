// 多信号波形：位置 / 速度 / 参考 / 误差
// 交互：滚轮缩放 Y，左键拖动平移 X，按钮重置。

const COLORS = ['#4da3ff', '#3fb950', '#d29922', '#f85149', '#a371f7', '#39c5cf'];
const WINDOW_SEC = 30;

export class JointPlot {
  constructor(el, jointNames) {
    this.el = el;
    this.jointNames = jointNames;
    this.signal = 'pos';
    this.visible = jointNames.map(() => true);
    this.yManual = null;     // {min,max}，null 表示自动
    this.panning = false;

    this.history = {
      t: [],
      pos: jointNames.map(() => []),
      vel: jointNames.map(() => []),
      ref: jointNames.map(() => []),
      err: jointNames.map(() => []),
    };

    this.uplot = new uPlot(this.options(), this.buildData(), el);
    this.bindInteractions();
    new ResizeObserver(() => this.resize()).observe(el);
  }

  options() {
    return {
      width: this.el.clientWidth || 600,
      height: this.el.clientHeight || 160,
      padding: [6, 10, 0, 0],
      cursor: { drag: { x: false, y: false } },   // 自己实现拖动平移
      scales: { x: { time: true }, y: { auto: true } },
      legend: { show: false },
      axes: [
        { stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' } },
        { stroke: '#8b98a9', grid: { stroke: '#2b323c' }, ticks: { stroke: '#2b323c' }, size: 52 },
      ],
      series: [{ label: '时间' }].concat(
        this.jointNames.map((name, i) => ({
          label: name,
          stroke: COLORS[i % COLORS.length],
          width: 1.5,
          points: { show: false },
          show: this.visible[i],
        }))
      ),
    };
  }

  buildData() {
    const src = this.history[this.signal];
    return [this.history.t].concat(src);
  }

  resize() {
    const width = this.el.clientWidth;
    const height = this.el.clientHeight;
    if (width > 0 && height > 0) {
      this.uplot.setSize({ width, height });
    }
  }

  push(t, values) {
    this.history.t.push(t);
    for (const key of ['pos', 'vel', 'ref', 'err']) {
      const arr = this.history[key];
      for (let i = 0; i < arr.length; i += 1) {
        arr[i].push(values[key] ? values[key][i] : null);
      }
    }

    const limit = t - WINDOW_SEC;
    while (this.history.t.length && this.history.t[0] < limit) {
      this.history.t.shift();
      for (const key of ['pos', 'vel', 'ref', 'err']) {
        this.history[key].forEach((arr) => arr.shift());
      }
    }
    this.redraw();
  }

  redraw() {
    this.uplot.setData(this.buildData());
    if (this.yManual) {
      this.uplot.setScale('y', this.yManual);
    } else {
      this.uplot.setScale('y', { auto: true });
    }
  }

  setSignal(name) {
    this.signal = name;
    this.redraw();
    this.resetX();
  }

  toggleJoint(index) {
    this.visible[index] = !this.visible[index];
    this.uplot.setSeries(index + 1, { show: this.visible[index] });
  }

  resetX() {
    const t = this.history.t;
    if (!t.length) return;
    const now = t[t.length - 1];
    this.uplot.setScale('x', { min: now - WINDOW_SEC, max: now });
  }

  resetAxes() {
    this.yManual = null;
    this.uplot.setScale('y', { auto: true });
    this.resetX();
  }

  zoomY(factor) {
    const range = this.uplot.scales.y;
    const min = range.min ?? -180;
    const max = range.max ?? 180;
    const center = (min + max) / 2;
    const half = Math.max(0.5, ((max - min) / 2) * factor);
    this.yManual = { min: center - half, max: center + half };
    this.uplot.setScale('y', this.yManual);
  }

  // 滚轮缩放 Y，左键拖动平移 X
  bindInteractions() {
    const el = this.el;

    el.addEventListener('wheel', (event) => {
      event.preventDefault();
      const factor = event.deltaY > 0 ? 1.15 : 1 / 1.15;
      this.zoomY(factor);
    }, { passive: false });

    el.addEventListener('pointerdown', (event) => {
      if (event.button !== 0) return;
      const scale = this.uplot.scales.x;
      if (!scale.max) return;
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
      if (!this.panning) return;
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
