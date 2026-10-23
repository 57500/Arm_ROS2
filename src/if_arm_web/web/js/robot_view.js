// 3D 视图：加载 URDF、跟随关节角、末端的 6 自由度拖拽手柄。
// 只负责显示和"目标位姿"，不做 IK —— IK 由后端调 MoveIt 完成。
//
// 坐标约定：URDF 是 Z 向上，所以相机 up 设为 +Z，默认从 -Y 方向正视 base_link。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import URDFLoader from '../vendor/urdf/URDFLoader.js';

const HOME_CAMERA = new THREE.Vector3(0, -1.25, 0.45);
const HOME_TARGET = new THREE.Vector3(0, 0, 0.22);

export class RobotView {
  constructor(container, options = {}) {
    this.container = container;
    this.tipLink = options.tipLink || 'link6';
    this.jointNames = [];
    this.onGizmoChange = null;
    this.onDragEnd = null;
    this.robot = null;
    this.ee = null;

    this.currentQ = null;   // 实际显示用的关节角（平滑后）
    this.targetQ = null;    // 目标关节角（来自 /joint_states）

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x1b2129);

    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);          // Z 向上，和 URDF 一致
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
    grid.rotation.x = Math.PI / 2;   // 网格放到 XY 平面
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.12));

    this.target = new THREE.Object3D();
    this.scene.add(this.target);

    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.setSpace('world');
    this.gizmo.setMode('translate');
    this.gizmo.setSize(0.9);
    this.gizmo.visible = false;
    this.scene.add(this.gizmo);
    this.gizmo.attach(this.target);

    this.gizmo.addEventListener('dragging-changed', (event) => {
      this.orbit.enabled = !event.value;
      if (!event.value && this.onDragEnd) {
        this.onDragEnd();
      }
    });
    this.gizmo.addEventListener('objectChange', () => {
      if (this.onGizmoChange) {
        this.onGizmoChange();
      }
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
    this.renderer.setSize(w, h);   // 同时更新 CSS 尺寸，避免高 DPI 下画面被放大裁切
  }

  /** 回到正视 base_link 的初始视角 */
  resetView() {
    this.camera.position.copy(HOME_CAMERA);
    this.orbit.target.copy(HOME_TARGET);
    this.orbit.update();
  }


  loadURDF(xml, meshPackage) {
    if (!xml) {
      return;
    }
    const loader = new URDFLoader();
    loader.packages = { [meshPackage]: `/robot/${meshPackage}` };
    const robot = loader.parse(xml, '');

    if (this.robot) {
      this.scene.remove(this.robot);
    }
    this.robot = robot;
    this.robot.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = false;
        child.receiveShadow = false;
      }
    });
    this.scene.add(robot);

    this.ee = robot.getObjectByName(this.tipLink) || null;
    this.robot.updateMatrixWorld(true);
    this.snapToEE();
    this.gizmo.visible = true;
  }

  setJoints(values) {
    if (!this.robot || !values) {
      return;
    }
    this.jointNames.forEach((name, i) => {
      if (i < values.length) {
        this.robot.setJointValue(name, values[i]);
      }
    });
    this.robot.updateMatrixWorld(true);
    this.currentQ = values.slice();
    this.targetQ = values.slice();
  }

  /** 设定目标角，由渲染循环平滑逼近（/joint_states 高频更新时更顺） */
  setTargetJoints(values) {
    if (!values) {
      return;
    }
    if (!this.currentQ) {
      this.currentQ = values.slice();
    }
    this.targetQ = values.slice();
  }

  updateSmoothing() {
    if (!this.robot || !this.targetQ || !this.currentQ) {
      return;
    }
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
    if (changed) {
      this.jointNames.forEach((name, i) => {
        if (i < this.currentQ.length) {
          this.robot.setJointValue(name, this.currentQ[i]);
        }
      });
      this.robot.updateMatrixWorld(true);
    }
  }

  setMode(mode) {
    this.gizmo.setMode(mode === 'rotate' ? 'rotate' : 'translate');
  }

  /** 把拖拽手柄放到当前末端位姿上 */
  snapToEE() {
    if (!this.ee) {
      return;
    }
    this.ee.updateMatrixWorld(true);
    this.ee.matrixWorld.decompose(
      this.target.position,
      this.target.quaternion,
      this.target.scale
    );
    this.target.updateMatrixWorld(true);
    this.gizmo.attach(this.target);
  }

  /** 拖拽目标位姿（相对 base_link；机器人放在原点，所以直接用世界坐标） */
  getTargetPose() {
    return {
      pos: [this.target.position.x, this.target.position.y, this.target.position.z],
      quat: [
        this.target.quaternion.x,
        this.target.quaternion.y,
        this.target.quaternion.z,
        this.target.quaternion.w,
      ],
    };
  }

  /** 用数值设置手柄（用于回零等） */
  setTargetPose(pos, quat) {
    this.target.position.set(pos[0], pos[1], pos[2]);
    this.target.quaternion.set(quat[0], quat[1], quat[2], quat[3]);
    this.target.updateMatrixWorld(true);
  }
}
