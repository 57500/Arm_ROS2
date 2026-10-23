// 3D 视图：加载 URDF、跟随关节角、末端的 6 自由度拖拽手柄。
// 只负责显示和"目标位姿"，不做 IK —— IK 由后端调 MoveIt 完成。

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import URDFLoader from '../vendor/urdf/URDFLoader.js';

export class RobotView {
  constructor(container, options = {}) {
    this.container = container;
    this.tipLink = options.tipLink || 'link6';
    this.jointNames = options.jointNames || [];
    this.onGizmoChange = null;   // 拖动中回调（由 app.js 节流后发 IK）
    this.onDragEnd = null;
    this.robot = null;
    this.ee = null;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14171c);

    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
    this.camera.position.set(0.9, -0.9, 0.75);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.container.appendChild(this.renderer.domElement);

    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.set(0, 0, 0.25);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.08;
    this.orbit.update();

    const hemi = new THREE.HemisphereLight(0xffffff, 0x30353d, 1.6);
    this.scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 1.4);
    dir.position.set(1.2, 1.5, 1.8);
    this.scene.add(dir);

    const grid = new THREE.GridHelper(2, 20, 0x3a424e, 0x242a33);
    grid.position.z = 0;
    grid.rotation.x = Math.PI / 2;   // URDF 是 Z 向上，网格放到 XY 平面
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.12));

    // 拖拽手柄的代理对象
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
      this.renderer.render(this.scene, this.camera);
    });
  }

  resize() {
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
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
