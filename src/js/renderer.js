export class ArmRenderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.azimuth = -0.72;
    this.elevation = 0.56;
    this.zoom = 520;
    this.center = { x: 0, y: 0, z: 0.25 };
    this.drag = null;
    this.scene = null;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.bindControls();
    this.resize();
  }

  bindControls() {
    this.canvas.addEventListener("pointerdown", (event) => {
      this.drag = { x: event.clientX, y: event.clientY, azimuth: this.azimuth, elevation: this.elevation };
      this.canvas.setPointerCapture(event.pointerId);
    });
    this.canvas.addEventListener("pointermove", (event) => {
      if (!this.drag) return;
      this.azimuth = this.drag.azimuth + (event.clientX - this.drag.x) * 0.008;
      this.elevation = Math.max(-0.05, Math.min(1.35, this.drag.elevation + (event.clientY - this.drag.y) * 0.006));
      this.draw();
    });
    this.canvas.addEventListener("pointerup", () => { this.drag = null; });
    this.canvas.addEventListener("pointercancel", () => { this.drag = null; });
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.zoom = Math.max(220, Math.min(900, this.zoom * Math.exp(-event.deltaY * 0.001)));
      this.draw();
    }, { passive: false });
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    const ratio = Math.min(window.devicePixelRatio || 1, 2);
    this.canvas.width = Math.max(1, Math.round(rect.width * ratio));
    this.canvas.height = Math.max(1, Math.round(rect.height * ratio));
    this.ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
    this.width = rect.width;
    this.height = rect.height;
    this.draw();
  }

  project(point) {
    const x = point.x - this.center.x;
    const y = point.y - this.center.y;
    const z = point.z - this.center.z;
    const ca = Math.cos(this.azimuth);
    const sa = Math.sin(this.azimuth);
    const ce = Math.cos(this.elevation);
    const se = Math.sin(this.elevation);
    const horizontal = x * ca - y * sa;
    const depth0 = x * sa + y * ca;
    const vertical = z * ce - depth0 * se;
    const depth = z * se + depth0 * ce;
    return {
      x: this.width * 0.5 + horizontal * this.zoom,
      y: this.height * 0.55 - vertical * this.zoom,
      depth,
    };
  }

  line(a, b, color, width = 1, dash = []) {
    const pa = this.project(a);
    const pb = this.project(b);
    this.ctx.beginPath();
    this.ctx.setLineDash(dash);
    this.ctx.moveTo(pa.x, pa.y);
    this.ctx.lineTo(pb.x, pb.y);
    this.ctx.lineWidth = width;
    this.ctx.strokeStyle = color;
    this.ctx.stroke();
    this.ctx.setLineDash([]);
  }

  point(point, radius, fill, stroke = null) {
    const projected = this.project(point);
    this.ctx.beginPath();
    this.ctx.arc(projected.x, projected.y, radius, 0, Math.PI * 2);
    this.ctx.fillStyle = fill;
    this.ctx.fill();
    if (stroke) {
      this.ctx.strokeStyle = stroke;
      this.ctx.lineWidth = 1.5;
      this.ctx.stroke();
    }
  }

  label(point, text, color = "#94a3b8") {
    const p = this.project(point);
    this.ctx.fillStyle = color;
    this.ctx.font = "11px Inter, system-ui, sans-serif";
    this.ctx.fillText(text, p.x + 7, p.y - 7);
  }

  drawGrid() {
    for (let value = -0.8; value <= 0.8001; value += 0.1) {
      const major = Math.abs(Math.round(value * 10)) % 5 === 0;
      const color = major ? "rgba(100,116,139,.22)" : "rgba(100,116,139,.10)";
      this.line({ x: -0.8, y: value, z: 0 }, { x: 0.8, y: value, z: 0 }, color, major ? 1 : 0.6);
      this.line({ x: value, y: -0.8, z: 0 }, { x: value, y: 0.8, z: 0 }, color, major ? 1 : 0.6);
    }
    this.line({ x: 0, y: 0, z: 0 }, { x: 0.28, y: 0, z: 0 }, "#ef5b4c", 2);
    this.line({ x: 0, y: 0, z: 0 }, { x: 0, y: 0.28, z: 0 }, "#4ade80", 2);
    this.line({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0.28 }, "#38bdf8", 2);
    this.label({ x: 0.29, y: 0, z: 0 }, "X", "#ef5b4c");
    this.label({ x: 0, y: 0.29, z: 0 }, "Y", "#4ade80");
    this.label({ x: 0, y: 0, z: 0.29 }, "Z", "#38bdf8");
  }

  drawBox(zone) {
    const [x0, x1] = zone.x;
    const [y0, y1] = zone.y;
    const [z0, z1] = zone.z;
    const corners = [
      { x: x0, y: y0, z: z0 }, { x: x1, y: y0, z: z0 },
      { x: x1, y: y1, z: z0 }, { x: x0, y: y1, z: z0 },
      { x: x0, y: y0, z: z1 }, { x: x1, y: y0, z: z1 },
      { x: x1, y: y1, z: z1 }, { x: x0, y: y1, z: z1 },
    ];
    for (const [a, b] of [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]]) {
      this.line(corners[a], corners[b], "rgba(45,212,191,.50)", 1, [5, 5]);
    }
  }

  update(scene) {
    this.scene = scene;
    this.draw();
  }

  draw() {
    if (!this.ctx || !this.width) return;
    const gradient = this.ctx.createLinearGradient(0, 0, 0, this.height);
    gradient.addColorStop(0, "#111c2b");
    gradient.addColorStop(1, "#08101b");
    this.ctx.fillStyle = gradient;
    this.ctx.fillRect(0, 0, this.width, this.height);
    this.drawGrid();
    if (!this.scene) return;

    this.drawBox(this.scene.zone);
    const workspace = [...(this.scene.workspace || [])].sort((a, b) => a.z - b.z);
    for (const sample of workspace) {
      const p = this.project(sample);
      this.ctx.globalAlpha = sample.safe ? 0.22 : 0.07;
      this.ctx.fillStyle = sample.safe ? "#2dd4bf" : "#f59e0b";
      this.ctx.fillRect(p.x, p.y, sample.safe ? 2 : 1.5, sample.safe ? 2 : 1.5);
    }
    this.ctx.globalAlpha = 1;

    for (const sample of this.scene.path || []) {
      this.point(sample.point, 2.1, sample.safe ? "rgba(56,189,248,.85)" : "rgba(239,91,76,.9)");
    }

    if (this.scene.target) {
      const t = this.scene.target;
      this.line({ x: t.x - 0.025, y: t.y, z: t.z }, { x: t.x + 0.025, y: t.y, z: t.z }, "#f8d267", 2);
      this.line({ x: t.x, y: t.y - 0.025, z: t.z }, { x: t.x, y: t.y + 0.025, z: t.z }, "#f8d267", 2);
      this.line({ x: t.x, y: t.y, z: t.z - 0.025 }, { x: t.x, y: t.y, z: t.z + 0.025 }, "#f8d267", 2);
      this.label(t, "目标", "#f8d267");
    }

    const points = this.scene.fk.points;
    const armColor = this.scene.safe ? "#e8edf5" : "#ff7b6f";
    for (let index = 0; index < points.length - 1; index += 1) {
      this.line(points[index], points[index + 1], "rgba(0,0,0,.5)", index === 0 ? 13 : 11);
      this.line(points[index], points[index + 1], armColor, index === 0 ? 9 : 7);
    }
    points.forEach((p, index) => this.point(p, index === points.length - 1 ? 7 : 6, index === 0 ? "#64748b" : "#38bdf8", "#dce7f3"));
    this.point(points.at(-1), 4, this.scene.safe ? "#4ade80" : "#ef5b4c");
    this.label(points.at(-1), "TCP", this.scene.safe ? "#4ade80" : "#ef5b4c");
  }
}

