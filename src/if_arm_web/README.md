# if_arm_web — 浏览器上位机

在浏览器里看 3D 模型、拖末端、看波形、急停。**不直接给电机发指令**，所有运动都走
MoveIt 和 ros2_control 控制器。

## 启动

```bash
# 1) 先起机械臂本体（控制器 + 硬件）
ros2 launch if_arm_lite_moveit_config demo.launch.py

# 2) 再起上位机
ros2 launch if_arm_web web.launch.py

# 3) 浏览器打开
http://127.0.0.1:8000/
```

端口可用参数改：`http_port:=8080 ws_port:=8081`。

## 界面

| 区域 | 内容 |
|---|---|
| 左侧 3D | URDF 模型，跟随 `/joint_states`；末端有拖拽手柄 |
| 右上 状态 | 控制器状态、硬件组件状态 |
| 右中 关节表 | 位置（度）、速度（度/秒） |
| 拖拽 | 平移 / 旋转 两种模式；拖动后机器人按 IK 跟随 |
| 执行 | 速度/加速度缩放、执行预览位姿、回零、急停 |
| 底部 波形 | 6 个关节位置，最近 30 秒 |

## 通信

```
浏览器 --HTTP--> 桥接节点(:8000)  静态文件 + /robot/<pkg>/... 网格
浏览器 --WebSocket--> 桥接节点(:8001)
桥接节点 <--rclpy--> MoveIt / controller_manager
```

不用 rosbridge，也不用 node/npm：前端是原生 ES Module，three.js / urdf-loader / uPlot
都在 `web/vendor/` 里（离线可用）。

### 消息

服务端 → 浏览器：

| type | 内容 |
|---|---|
| `hello` | 关节名、URDF、group/tip、初始关节角 |
| `urdf` | URDF 后到时补发 |
| `state` | `q`、`dq`、时间戳，50 Hz |
| `status` | 控制器与硬件组件状态，1 Hz |
| `ik` | `seq`、`ok`、`q` 或 `error` |
| `execute` / `estop` / `log` | 执行结果、急停结果、日志 |

浏览器 → 服务端：`ik`（含 `pos`/`quat`/`seed`/`seq`）、`execute`、`estop`、`ping`。

## 和 RViz 拖拽一致吗

- IK 调的是 MoveIt 的 `/compute_ik`，和 RViz 用同一个求解器（`kinematics.yaml` 里的 KDL）
- 种子用当前关节角；拖动过程中用上一次的解作为下一次的种子（RViz 也是这样，避免翻腕）
- **多解是物理事实**，无法保证"与 RViz 手动拖出的角度逐位相同"；但同样的输入（同种子、
  同选项）结果确定（已实测：同种子重复调用偏差 0）
- 真正保证的是 **预览 = 执行**：执行时把 IK 解出来的 6 个关节角作为 `JointConstraint`
  发给 `/move_action`，MoveIt 不会再做一次 IK，所以不会跑到另一组解

### 关于 `ik_avoid_collisions`

默认 **false**。原因：当前 SRDF 的 `disable_collisions` 覆盖不全，`avoid_collisions=true`
时很多正常位姿会被判成碰撞而返回 `-31`（实测）。

关闭它只影响 **拖动预览的响应**；点"执行"后 MoveIt 规划轨迹时仍然做碰撞检查，
所以执行安全性不变。需要严格预览时可以：

```bash
ros2 launch if_arm_web web.launch.py
# 或者单独跑节点
ros2 run if_arm_web bridge.py --ros-args -p ik_avoid_collisions:=true
```

## 安全边界

| 动作 | 走谁 |
|---|---|
| 拖拽 IK | MoveIt `/compute_ik` |
| 执行 | MoveIt `/move_action`（规划 + 执行） |
| 速度缩放 | MoveIt `max_velocity_scaling_factor` |
| 急停 | 取消目标 → 停控制器 → 硬件组件切 inactive（触发 0 速度 + 关 CAN） |
| 限速兜底 | `joint_limits.yaml` + 硬件 `max_joint_vel` + 驱动 rpm 上限 |

上位机不写任何插值、不限速、不发 `F6`。

## 已知限制

1. **前端没有在真实浏览器里跑过**（开发环境没有浏览器）。已验证：所有 JS/CSS/库
   都能通过 HTTP 取到、HTML 里被 JS 引用的 19 个 id 全部存在、ES Module 的 import
   全部可解析、WS 协议与 IK 后端实测通过。
2. `execute` 需要 controller_manager 在跑；只起 `move_group` 时返回 `-4`（CONTROL_FAILED），
   这是预期行为。
3. 急停里的"停控制器/停硬件"依赖 `controller_manager` 服务，不在线时会跳过并返回说明。
4. SRDF 的碰撞对配置不全，见上面 `ik_avoid_collisions`。

## 验证记录（电机无关部分）

| 检查 | 结果 |
|---|---|
| `colcon build` | 通过 |
| HTTP `/`、JS、CSS、three、uPlot、`/robot/if_arm_lite/meshes/link6.STL` | 全部 200 |
| `/config.json` | `{"ws_port": 8001}` |
| WS `hello` | 关节名正确，URDF 8798 字节（从 move_group 参数回退读取） |
| IK 闭环 | 用 FK(seed) 的位姿做 IK，返回的关节角与 seed 偏差 0.0 |
| 确定性 | 同种子重复调用偏差 0.0 |
| 执行（无控制器） | 返回 MoveIt `-4`，进程不崩 |
| 急停（无控制器） | 返回"没有可执行的动作"，进程不崩 |

真机验收时重点看：拖动是否跟手、预览与执行是否一致、急停是否真的停车。
