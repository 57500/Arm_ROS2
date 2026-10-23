// 关节位置波形（uPlot，最近 30 秒滚动）

const COLORS = ['#4da3ff', '#3fb950', '#d29922', '#f85149', '#a371f7', '#39c5cf'];

export class JointPlot {
  constructor(el, jointNames) {
    this.el = el;
    this.jointNames = jointNames;
    this.windowSec = 30;
    this.data = [[]].concat(jointNames.map(() => []));

    const series = [{ label: '时间' }].concat(
      jointNames.map((name, i) => ({
        label: name,
        stroke: COLORS[i % COLORS.length],
        width: 1.4,
        points: { show: false },
      }))
    );

    this.uplot = new uPlot(
      {
        width: el.clientWidth || 600,
        height: el.clientHeight || 160,
        padding: [6, 10, 0, 0],
        cursor: { drag: { x: true, y: false } },
        scales: { x: { time: true }, y: { auto: true } },
        legend: { show: true },
        axes: [
          {
            stroke: '#8b98a9',
            grid: { stroke: '#2b323c' },
            ticks: { stroke: '#2b323c' },
          },
          {
            stroke: '#8b98a9',
            grid: { stroke: '#2b323c' },
            ticks: { stroke: '#2b323c' },
            size: 52,
          },
        ],
        series,
      },
      this.data,
      el
    );

    new ResizeObserver(() => this.resize()).observe(el);
  }

  resize() {
    const width = this.el.clientWidth;
    const height = this.el.clientHeight;
    if (width > 0 && height > 0) {
      this.uplot.setSize({ width, height });
    }
  }

  /** t: 秒（unix 时间），q: 弧度数组 */
  push(t, q) {
    const deg = q.map((v) => (v * 180) / Math.PI);
    this.data[0].push(t);
    deg.forEach((v, i) => {
      if (this.data[i + 1]) {
        this.data[i + 1].push(v);
      }
    });

    const limit = t - this.windowSec;
    while (this.data[0].length && this.data[0][0] < limit) {
      this.data.forEach((arr) => arr.shift());
    }
    this.uplot.setData(this.data);
  }
}
