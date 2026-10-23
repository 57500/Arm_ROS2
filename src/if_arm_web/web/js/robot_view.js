// 3D 视图：真实机械臂 + 半透明预览机械臂。
// 真实手臂跟随 /joint_states；预览手臂显示 IK/滑块的结果（拖动时可见）。
// 坐标约定：URDF 是 Z 向上，相机 up 设为 +Z。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import URDFLoader from '../vendor/urdf/URDFLoader.js';

const HOME_CAMERA = new THREE.Vector3(1.25, 0, 0.45);
const HOME_TARGET = new THREE.Vector3(0, 0, 0.22);
const GHOST_COLOR = new THREE.Color(0x4da3ff);

export class RobotView {
  constructor(container, options = {}) {
    this.container = container;
    this.tipLink = options.tipLink || 'link6';
    this.jointNames = [];
    this.onGizmoChange = null;
    this.onDragEnd = null;

    this.robot = null;   // 真实
    this.ghost = null;   // 预览
    this.ee = null;

    this.currentQ = null;
    this.targetQ = null;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1b2129);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.copy(HOME_CAMERA);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.container.appendChild(this.renderer.domElement);

    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.copy(HOME_TARGET);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.1;
    this.orbit.update();

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x30353d, 1.8));
    const dir = new THREE.DirectionalLight(0xffffff, 1.5);
    dir.position.set(1.2, -1.5, 1.8);
    this.scene.add(dir);

    const grid = new THREE.GridHelper(2, 20, 0x5a667a, 0x333c48);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.12));

    this.target = new THREE.Object3D();
    this.scene.add(this.target);

    // 平移 + 旋转两个手柄同时挂在同一目标上，可以直接拖任一个
    this.gizmoMove = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmoMove.setSpace('world');
    this.gizmoMove.setMode('translate');
    this.gizmoMove.setSize(0.85);
    this.gizmoMove.visible = false;
    this.gizmoMove.enabled = false;
    this.scene.add(this.gizmoMove);
    this.gizmoMove.attach(this.target);

    this.gizmoRotate = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmoRotate.setSpace('world');
    this.gizmoRotate.setMode('rotate');
    this.gizmoRotate.setSize(0.65);
    this.gizmoRotate.visible = false;
    this.gizmoRotate.enabled = false;
    this.scene.add(this.gizmoRotate);
    this.gizmoRotate.attach(this.target);

    // 去掉"自由旋转"的隐形拾取球/环，避免和移动箭头抢点击
    const rotatePicker = this.gizmoRotate._gizmo
      && this.gizmoRotate._gizmo.picker
      && this.gizmoRotate._gizmo.picker.rotate;
    if (rotatePicker) {
      rotatePicker.children.forEach((child) => {
        if (child.name === 'E' || child.name === 'XYZE') child.visible = false;
      });
    }

    [this.gizmoMove, this.gizmoRotate].forEach((gizmo) => {
      const other = gizmo === this.gizmoMove ? this.gizmoRotate : this.gizmoMove;
      gizmo.addEventListener('dragging-changed', (event) => {
        this.orbit.enabled = !event.value;
        other.enabled = !event.value;
        if (!event.value && this.onDragEnd) this.onDragEnd();
      });
      gizmo.addEventListener('objectChange', () => {
        if (this.onGizmoChange) this.onGizmoChange();
      });
    });

    new ResizeObserver(() => this.resize()).observe(this.container);
    this.resize();

    this.renderer.setAnimationLoop(() => {
      this.orbit.update();
      this.updateSmoothing();
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  resetView() {
    this.camera.position.copy(HOME_CAMERA);
    this.orbit.target.copy(HOME_TARGET);
    this.orbit.update();
  }

  loadURDF(xml, meshPackage) {
    if (!xml) return;

    const loader = new URDFLoader();
    loader.packages = { [meshPackage]: `/robot/${meshPackage}` };

    if (this.robot) this.scene.remove(this.robot);
    if (this.ghost) this.scene.remove(this.ghost);

    this.robot = loader.parse(xml, '');
    this.robot.traverse((child) => {
      if (!child.isMesh) return;
      // 真实手臂必须是实体:若 URDF 材质带透明度,这里强制不透明
      const solidify = (mat) => {
        if (!mat.transparent && mat.opacity === 1) return mat;
        const m = mat.clone();
        m.transparent = false;
        m.opacity = 1;
        m.depthWrite = true;
        m.needsUpdate = true;
        return m;
      };
      child.material = Array.isArray(child.material)
        ? child.material.map(solidify)
        : solidify(child.material);
      child.renderOrder = 0;
    });
    this.scene.add(this.robot);

    // 预览手臂:半透明蓝色幽灵
    this.ghost = loader.parse(xml, '');
    this.ghost.traverse((child) => {
      if (!child.isMesh) return;
      const paint = (mat) => {
        const m = mat.clone();
        m.transparent = true;
        m.opacity = 0.28;
        m.depthWrite = false;
        m.side = THREE.DoubleSide;
        if (m.color) m.color.lerp(GHOST_COLOR, 0.7);
        if (m.emissive) m.emissive.lerp(GHOST_COLOR, 0.35);
        m.needsUpdate = true;
        return m;
      };
      child.material = Array.isArray(child.material)
        ? child.material.map(paint)
        : paint(child.material);
      child.renderOrder = 10;
    });
    this.ghost.visible = false;
    this.scene.add(this.ghost);

    this.ee = this.robot.getObjectByName(this.tipLink) || null;
    this.robot.updateMatrixWorld(true);
    this.snapToEE();
    this.gizmoMove.visible = true;
    this.gizmoMove.enabled = true;
    this.gizmoRotate.visible = true;
    this.gizmoRotate.enabled = true;
  }

  applyJoints(robot, values) {
    if (!robot || !values) return;
    this.jointNames.forEach((name, i) => {
      if (i < values.length) robot.setJointValue(name, values[i]);
    });
    robot.updateMatrixWorld(true);
  }

  /** 真实手臂：立即设置（初始化用） */
  setJoints(values) {
    this.applyJoints(this.robot, values);
    this.currentQ = values ? values.slice() : null;
    this.targetQ = values ? values.slice() : null;
  }

  /** 真实手臂：平滑逼近（/joint_states 高频更新） */
  setTargetJoints(values) {
    if (!values) return;
    if (!this.currentQ) this.currentQ = values.slice();
    this.targetQ = values.slice();
  }

  /** 预览手臂：显示 IK/滑块结果 */
  setPreview(values) {
    if (!this.ghost || !values) return;
    this.ghost.visible = true;
    this.applyJoints(this.ghost, values);
  }

  clearPreview() {
    if (this.ghost) this.ghost.visible = false;
  }

  updateSmoothing() {
    if (!this.robot || !this.targetQ || !this.currentQ) return;
    let changed = false;
    const n = Math.min(this.targetQ.length, this.currentQ.length);
    for (let i = 0; i < n; i += 1) {
      const diff = this.targetQ[i] - this.currentQ[i];
      if (Math.abs(diff) > 1e-4) {
        this.currentQ[i] += diff * 0.4;
        changed = true;
      } else if (diff !== 0) {
        this.currentQ[i] = this.targetQ[i];
        changed = true;
      }
    }
    if (changed) this.applyJoints(this.robot, this.currentQ);
  }

  snapToTip(useGhost) {
    const root = useGhost ? this.ghost : this.robot;
    const ee = root ? root.getObjectByName(this.tipLink) : null;
    if (!ee) return;
    root.updateMatrixWorld(true);
    ee.updateMatrixWorld(true);
    ee.matrixWorld.decompose(this.target.position, this.target.quaternion, this.target.scale);
    this.target.updateMatrixWorld(true);
    this.gizmoMove.attach(this.target);
    this.gizmoRotate.attach(this.target);
  }

  snapToEE() {
    this.snapToTip(false);
  }

  snapToPreviewEE() {
    this.snapToTip(true);
  }

  getTargetPose() {
    return {
      pos: [this.target.position.x, this.target.position.y, this.target.position.z],
      quat: [
        this.target.quaternion.x, this.target.quaternion.y,
        this.target.quaternion.z, this.target.quaternion.w,
      ],
    };
  }

  setMode() {
    // 平移和旋转手柄现在同时显示，不需要切换
  }
}
