# Y7 机械臂 ROS2 学习工程规划（教学文档）

> **文档定位**：本项目的「学习路线 + 教学计划 + 变更日志」唯一真相源。
> - `PROJECT_STATUS.md` = 你的**技术笔记与踩坑记录**（概念总结、Assistant 缺陷、报错排查过程），继续保留，不用改。
> - **本文件** = **做什么、按什么顺序做、怎么验证、做完了没**，以及每次改动的留痕。
>
> **约定**
> - ✅ 已完成 ｜ 🚧 进行中 ｜ ⬜ 未开始 ｜ ❌ 已否决（记录原因，避免重复讨论）
> - 每完成一项，勾选并在 §6 变更日志追加一行。
>
> **当前阶段**：D1 已收尾 → 下一步 D2（见 §4）
> **最后更新**：2026-09-12（D1 工程清理完成）

---

## 0. 怎么用这份文档

| 角色 | 用法 |
| --- | --- |
| 你（学生） | 按 §4 顺序推进；卡住时把**报错原文 + 验证命令输出**贴出来，不要只说"不行" |
| 我（AI 老师） | 只在 §4 框架内教学；改动后更新 §5/§6；不主动写控制代码，代码由你敲 |
| 复习 | 看 §3（概念映射）、§5（陷阱清单）、§6（变更日志）三节即可 |

**教学原则（双方遵守）**
1. **你手写代码**，我只给「看哪个文件的哪一段 + 用什么命令验证 + 完成标准」。
2. **不学没有价值的**：官方现成件能解决的，绝不自己造（见 §1.3）。
3. **每个概念都要有可执行的验证手段**，没有验证命令的知识点不算学会。
4. **不许让 `install/` 成为真相源**（见 §5 T1）。

---

## 1. 目标与边界

### 1.1 总目标
```
RViz/MoveIt 规划 → 中间控制层(整形/补偿) → ros2_control 硬件接口 → CAN → 电机
```

### 1.2 阶段目标
| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 阶段1 | 仿真闭环：URDF + MoveIt + mock_components | ✅ |
| 阶段2 | 中间控制层：轨迹接收/整形/补偿 | 🚧 骨架完成，待健壮化 |
| 阶段3 | 底层通信：CAN / MCU（SystemInterface 插件） | ⬜ |
| 阶段4 | 真机：标定、限速、闭环验证 | ⬜ |

### 1.3 明确**不做**
| 不做 | 理由 |
| --- | --- |
| 自己写 controller plugin 替代 `joint_trajectory_controller` | 官方 jtc 足够；openarm 也没自己写 |
| 深挖 Gazebo | `mock_components` 已达同等验证目的 |
| 读 MoveIt 规划器/流水线源码 | 用现成插件 + 调参足够 |
| ROS1 / 自定义 msg（现阶段） | 与目标无关 |
| **只做"插值"的中间层** | jtc 自带 `splines` 插值，重复造 = 无价值（见 D5） |

### 1.4 中间控制层的**真正价值定位**（重要）
不是"把 10Hz 插值成 100Hz"（官方已做），而是：
- 动力学前馈补偿（重力/科氏/摩擦）→ **需要开 `effort` 接口**
- 限幅与速度整形、轨迹平滑、安全监控（误差超限急停）
- 多控制器切换、与力/视觉反馈融合

---

## 2. 资产盘点（D1 清理后）

### 2.1 保留（`src/` 下只剩 3 个包）
| 包 | 内容 | 结论 |
| --- | --- | --- |
| `y7_arm_lite` | SolidWorks 导出的 URDF + meshes | ✅ 保留。**URDF 只有惯性，无阻尼/摩擦；且 base_link 带惯性**（见 §5 T3） |
| `y7_moveit_config` | Assistant 生成的 MoveIt 配置 | ✅ 保留。6 个 Assistant 缺陷需每次重新生成后手修 |
| `arm_control_layer` | 中间控制层（5 个 Python 脚本） | ✅ 保留，待 D2 健壮化 + D3 加 launch |

### 2.2 已归档到 `57500/archive/`（不再参与构建）
| 文件 | 归档理由 |
| --- | --- |
| `arm_description/` | 早期练习包，已被 `y7_arm_lite` 取代 |
| `y7_arm_lite/launch/display.launch`、`gazebo.launch` | **ROS1 语法**（`$(find ...)`），Jazzy 下不可能运行 |
| `y7_arm_lite/export.log` | SolidWorks 导出日志（648KB），非运行所需 |
| `archive/COLCON_IGNORE` | 标记该目录不被 colcon 扫描（**关键**，见 §5 T2） |

> 归档而非删除：需要哪个文件，手动移回 `src/` 即可。

### 2.3 已删除
| 对象 | 大小 | 理由 |
| --- | --- | --- |
| `y7_arm_lite/.vscode/browse.vc.db` | **1.1 GB** | VS Code C++ 索引缓存，非工程文件，会自动重建 |
| `y7_arm_lite/textures/` | 空 | 无内容 |
| `build/ install/ log/` | — | 全量清理后重建，清除陈旧残留（见 §5 T1） |
| `install/.../config/ompl_planning.yaml` | 635 B | **多余文件**，内容与 MoveIt 官方默认逐字节相同，已实测不需要（见 §6 C1） |
| `install/.../config/sensors_3d.yaml` | 38 B | 内容仅 `sensors: []`，留着只会误导 |
| `install/arm_description/` | — | 陈旧包残留（`--symlink-install` 不会自动清理） |

### 2.4 待修问题清单
| # | 位置 | 问题 | 阶段 |
| --- | --- | --- | --- |
| P1 | URDF | `base_link` 带惯性 → KDL 警告，影响运动学与**以后的动力学** | D2 |
| P2 | `trajectory_proxy.py` | 无 cancel 转发、无多 goal 抢占处理 | D2 |
| P3 | `trajectory_proxy.py` | `execute_callback` 里 `while not done: sleep()` 忙等 | D2 |
| P4 | `arm_control_layer` | 没有 launch，5 个脚本手动起 | D3 |
| P5 | `y7_moveit_config` | package.xml 无 `exec_depend arm_control_layer`；无 bringup 包串联 | D3 |
| P6 | `ros2_controllers.yaml` | 缺 `interpolation_method`/`constraints`/`state_publish_rate`；`update_rate: 100` 偏低 | D3 |
| P7 | `kinematics.yaml` | `timeout: 0.005`（5ms）对 KDL 过短，易"假失败" | D2 |
| P8 | `joint_limits.yaml` + URDF `<limit>` | `velocity: 2.0`/`accel: 1.0` 是**占位值**，非电机真实指标 | 真机前 |
| P9 | `trajectory_feedback_listener.py:28` | 打印标题写在 for 循环内，7 个关节刷 7 遍 | D2 |
| P10 | `joint_state_listener.py:24` | `zip()` 未防长度不等 / velocity 为空 | D2 |
| P11 | `~/.bashrc` | source 了两个不存在的工作空间（`ros2_ws`/`ros2_study`），每次开 shell 报错 | D1（你决定） |

---

## 3. 目标架构 与 概念映射

### 3.1 目标架构（阶段3 完成后）

```
┌─────────────── PC (Linux, 非硬实时) ───────────────┐
│  RViz ──▶ move_group ──(action)──▶ 中间控制层      │
│                                      │ 整形/补偿    │
│                                      ▼ (action)    │
│  controller_manager (RT 线程, prio 50)             │
│    ├─ joint_state_broadcaster  ──▶ /joint_states   │
│    └─ manipulator_controller (jtc)                 │
│           │ command_interface (内存直写)            │
│           ▼                                        │
│      你的 SystemInterface 插件  ←── read()/write() │
│           │                                        │
└───────────┼────────────────────────────────────────┘
            ▼  CAN / CAN-FD
      电机驱动器 (或先经 MCU 做 1kHz 内环，PC 只给 50Hz 前馈)
```

**关键判断（写死）**：CAN 通信**必须**做成 `hardware_interface::SystemInterface` 插件，**不能**做成独立 Python 节点去发指令，否则 ros2_control 就变成摆设。
参考：`openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp`。

### 3.2 概念映射：ROS2 ↔ 你已有的 STM32/FreeRTOS 经验

> 教学时会优先用右列的**你已经写过的东西**来解释左列，差异处重点讲。
> 你的参考仓库：`/home/a57500/workspace/openarm/MechanicalArm_Code_V4`

| ROS2 概念 | 你 STM32 工程里的对应物 | 关键差异（重点理解） |
| --- | --- | --- |
| **Node** | FreeRTOS Task（`PTP_Task`/`PAD_Task`/`Uart_Task`） | Node 是独立进程，默认多线程跑在 `Executor` 里；Task 是同一地址空间的调度单元 |
| **Executor / callback_group** | FreeRTOS 调度器 + 任务优先级 | 回调之间是否互相阻塞由 callback_group 决定；对应你用信号量/队列控制任务切换 |
| **Topic + QoS** | `xQueueSend/Receive`、全局 `RobotState` | Topic 跨进程/跨机器、可一对多、有历史深度与可靠性策略（QoS） |
| **`/joint_states`** | `RobotState.h` 里的关节角全局量 | ROS2 里状态必须**被广播**，显示/规划/控制器全靠订阅 |
| **Action（Goal/Feedback/Result/Cancel）** | `PTP:x,y,z,...` 指令 + 完成回执 | Action 天然支持**异步 + 进度反馈 + 取消** |
| **Service** | 串口一问一答的握手指令 | 同步阻塞、无进度、不可取消 |
| **`MotorControl::Motor_PosControl` 打包帧** | — | 对应 `SystemInterface::write()`：命令值 → 报文 |
| **`MyCAN1_Transmit` / CAN 接收** | — | 对应 `read()`：报文 → 状态值 |
| **`S_Curve_Profile`（1024 点表 + `CONTROL_DT=0.01`）** | — | 对应 jtc 的插值 + `update_rate`（你当前 100Hz） |
| **`Inverse_Kinematics.cpp`（931 行 DH 解法）** | — | 对应 KDL 插件（`kinematics.yaml`）——官方件，不必自己写 |
| **`Forward_Kinematics` + 位姿** | — | 对应 `robot_state_publisher` 发 TF |
| **`PadControl` 的 Standby/Cartesian/Joint/Tool 状态机** | — | 对应 MoveIt Servo / 遥控节点；模式切换 = 控制器切换 |
| **`Stop_Flag` + `Stop_Semaphore` + `Emergency_Stop()`** | — | 对应 Action cancel + `on_deactivate()`；**真机安全核心** |
| **`NewBee` 同步广播/协议层** | — | 对应你自己的 CAN 协议封装（阶段3 要重写） |
| **硬实时 1kHz（TIM1 ISR）** | — | PC 侧只有 controller_manager RT 线程（日志可见 `FIFO RT scheduling priority: 50`）；要 1kHz 内环则 **MCU 不能省** |

**结论**：你原来"PC 低频前馈 + MCU 1kHz 插值闭环"的规划是**合理的**。openarm 的做法是让 PC 直接以 750Hz 跑控制回路（`openarm_bimanual_controllers.yaml` 里 `update_rate: 750`）。**这个架构选择要在 D6 之前定下来**（见 §8 Q1/Q2）。

---

## 4. 阶段计划

### D1 ✅ 工程清理（已完成，2026-09-12）
**目标**：把学习过程留下的垃圾清掉，让项目可回滚、`install/` 不再是真相源。
**知识点**：colcon 构建产物与真相源的关系、`--symlink-install` 的副作用、包结构规范、`COLCON_IGNORE`。
**做了什么**：见 §2.2/2.3 与 §6 C1。
**验证命令**（可复现）
```bash
cd /home/a57500/workspace/openarm/57500
du -sh .                     # 期望 ~1.3M（清理前 >1.1G）
ls install                   # 期望不含 arm_description
git status --short
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
# 期望：Summary: 3 packages finished
```
**完成标准**：✅ 3 个包干净构建；`install/` 无冗余文件。

---

### D2 ⬜ `trajectory_proxy` 健壮化 + URDF 小修
**目标**：让中间层在"取消 / 抢占 / 忙等"三种情况下行为正确。
**知识点**
- Action 完整状态机：`ACCEPTED → EXECUTING → (CANCELING) → SUCCEEDED/ABORTED/CANCELED`，及 `goal_handle.canceled()` 等 API。
- `cancel_callback` 的语义："收到取消后要不要转发给下游"。
- **抢占（preemption）**：新 goal 到来时旧 goal 必须被终止（MoveIt 会连点 Plan&Execute）。
- 回调链（`add_done_callback`）替代 busy-wait；`MultiThreadedExecutor` + callback_group 的取舍。
**参考**
- 你**已经写对**的样板：`arm_control_layer/scripts/trajectory_action_client.py`（回调链）
- 待改：`arm_control_layer/scripts/trajectory_proxy.py`
- 对照：`openarm_bimanual_controllers.yaml` 的 `constraints.goal_time`/`stopped_velocity_tolerance`（下游怎么判失败）
**你要做的**
1. 给 ActionServer 加 `cancel_callback`，把取消转发给下层 jtc。
2. 用 `self.active_goal_handle_` 记录当前 goal，新 goal 来时取消旧的。
3. 把 `while not future.done(): sleep(0.01)` 改成 `add_done_callback` 链。
4. 顺手修 P9/P10/P7（打印位置、zip 防护、KDL timeout → 0.1）。
**验证命令**
```bash
ros2 launch y7_moveit_config demo.launch.py use_rviz:=false   # 终端A
ros2 run arm_control_layer trajectory_proxy                   # 终端B
ros2 action info /trajectory_proxy/follow_joint_trajectory -t
ros2 topic hz /manipulator_controller/controller_state
```
**完成标准**：执行中取消 → 下层立即停止且 proxy 返回 `CANCELED`；连续两次 Execute 不叠加运动；CPU 不再因 busy-wait 飙升。

---

### D3 ⬜ launch 与 bringup 工程化
**目标**：一条命令起全链路，消灭"必须记得先开 proxy"的隐性依赖。
**知识点**
- launch 三件套：`DeclareLaunchArgument` / `Node(parameters=[...])` / `IncludeLaunchDescription`。
- **参数注入 vs 话题传递**：openarm 用 `parameters=[{'robot_description': ...}]`；你的 demo.launch 用 `remappings=[('/controller_manager/robot_description','/robot_description')]`。两条路都要能讲清（启动日志里 `Subscribing to '/robot_description' topic` 就是后者在生效）。
- `TimerAction` / `OpaqueFunction` 解决启动顺序；命名空间与 `spawner --controller-manager` 寻址。
**参考（精读，只读不抄）**
- `openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py`
- `openarm_ros2/openarm_bringup/config/controllers/*.yaml`
- 你的 `y7_moveit_config/launch/spawn_controllers.launch.py`
**你要做的**
1. `arm_control_layer/launch/control_layer.launch.py`：起 proxy（+ 可选监听器）。
2. 新建 `y7_bringup` 包：rsp + ros2_control_node + proxy + 两个 spawner + rviz，带 `use_rviz`/`use_proxy` 参数。
3. 给 `y7_bringup` 的 package.xml 写全 `exec_depend`。
4. 补 P6：`ros2_controllers.yaml` 加 `interpolation_method`、`state_publish_rate`、`action_monitor_rate`、`constraints`（先读懂 openarm 的注释再决定填什么）。
**验证命令**
```bash
ros2 launch y7_bringup bringup.launch.py
ros2 control list_controllers                               # 两个都 active
ros2 action list | grep -E 'proxy|manipulator'              # proxy 有 client（move_group 连上）
ros2 param get /controller_manager enforce_command_limits
```
**完成标准**：一条命令启动后 RViz 里 Plan&Execute 直接可用。

---

### D4 ⬜ 诊断工具箱 + ros2 bag + QoS
**目标**：从"跑起来"进化到"看得见、可复现、能回放"。
**知识点**
- 诊断命令全家桶：`node info` / `topic info -t` / `topic hz` / `topic echo --once` / `action info` / `param dump` / `interface show` / `doctor`。
- **QoS**：`reliable` vs `best_effort`、`transient_local`（`/robot_description` 靠它让后启动的进程也能拿到）、`durability` 对订阅时机的影响。
- `ros2 bag record`/`play` 与 `--clock`（事后回放调试）。
**你要做的**
1. 把常用诊断命令整理进 `PROJECT_STATUS.md`（跑通后自己总结，不要抄）。
2. 录一段 bag：`/joint_states` + `/manipulator_controller/controller_state` + TF，再 `ros2 bag play` 观察 RViz 复现运动。
3. QoS 实验：把 `/joint_states` 订阅者改成 `transient_local`，观察差异并解释。
**验证命令**
```bash
ros2 topic info /joint_states --verbose        # 看 QoS
ros2 bag record -o /tmp/run1 /joint_states /tf
ros2 bag info /tmp/run1
```
**完成标准**：能独立解释 `/robot_description` 为什么必须 `transient_local`；能录能放一段轨迹。

---

### D5 ⬜ 想清楚中间层"补什么"（设计日，不写代码）
**目标**：定下补偿算法的**数学形式**与**接口形式**，避免做出"重复 jtc"的东西。
**知识点**
- jtc 已提供：`splines` 插值、`update_rate` 周期执行、`constraints` 判失败。
- 中间层必须增加**新信息**：前馈力矩、受限速度剖面、轨迹再整形（S 曲线 / jerk 限制）。
- **你的既有资产**：`Trajectory_Planning::S_Curve_Profile`（S 曲线 + 速度上限 + PID）——它就是你的中间层雏形，把它"翻译"到 ROS2 是本阶段的关键动作。
- 前提：URDF inertial 是否可信。你的 `y7_arm_lite.csv` 有 CAD 的质量/质心/惯量（连杆 0.165~0.188 kg），看起来是真实值 → 好消息。
**纸面产出（不是代码）**
1. 中间层输入/输出接口表（订阅什么、发布什么、消息类型）。
2. 补偿项清单：重力（必做）/ 摩擦（可选）/ 惯量前馈（后做），各自需要哪些参数。
3. 决定**力矩往哪里去**：jtc 的 `effort` 前馈？还是自己走 MIT 模式？→ 直接决定 §8 Q1/Q2。
**完成标准**：一页纸接口设计 + 一个最小验证（哪怕先用假值加重力偏置）。

---

### D6–D7 ⬜ socketcan 脱机练手（vcan）
**目标**：没有硬件也能把 CAN 收发、帧结构、字节序练熟。
**知识点**
- Linux SocketCAN：`ip link` 配置、`vcan` 虚拟总线、`candump`/`cansend`/`cansniffer`。
- CAN 帧结构：11/29 位 ID、DLC、8 字节数据、大端/小端、标准帧 vs 扩展帧、CAN-FD 与 BRS。
- `python-can` 的 `Bus`/`Message`/`Notifier` 三种收包方式。
- **注意**：你的 STM32 项目是**步进驱动器（ZDT）CAN 协议**，与达妙电机完全不同 → 以你自己的驱动器手册为准，openarm 的 `dm_motor_control.cpp` 只作"位域打包"阅读材料。
**参考**
- `openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp`（位域打包思路）
- `openarm_can/python/examples/test_posvel.py`（最小验证脚本范式）
- 你 STM32 的 `MotorControl.cpp`（ZDT 帧：`Addr + 0xF6 + Dir + Rpm_H + Rpm_L + Acc + Syn + 0x6B`）
**你要做的**
1. 起 vcan：`sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0`。
2. 用 `candump`/`cansend` 手工收发，理解帧。
3. 写一个**与 ROS 无关**的 python-can 程序：周期发、收帧解析、打印。
4. 写纯函数把"关节弧度 → 驱动器报文"打包（对应 `MotorControl::Motor_PosControl`），**先不管 ROS**。
**验证命令**
```bash
sudo ip link show vcan0
candump vcan0                      # 终端A
cansend vcan0 001#F60103E8006B     # 终端B（按你的帧格式改）
python3 can_demo.py
```
**完成标准**：能解释"为什么 Rpm 要拆高低字节""同步帧 Syn 的作用"；脱机程序 100Hz 稳定收发。

---

### D8–D10 ⬜ SystemInterface 插件（先用 vcan 假电机）
**目标**：把 D6–D7 的收发函数装进 ros2_control，让 MoveIt 驱动"假电机"。
**知识点**
- `SystemInterface` 生命周期：`on_init → on_configure → on_activate → read()/write() → on_deactivate → on_cleanup`。
- `export_state_interfaces()`/`export_command_interfaces()` 返回的**指针**语义（控制器直接写你的成员变量）。
- `hardware_parameters`（xacro 的 `<hardware>` 里传 `can_interface`）+ 用 `ros2 param get` 验证。
- `pluginlib` 导出（`PLUGINLIB_EXPORT_CLASS`）+ package.xml 的 `export` 段（参考 `openarm_hardware.xml`）。
- 真机安全：`return_to_zero()`（上电渐变到零位，openarm 用 200 步 × 10ms）。
**参考**：`openarm_hardware/{openarm_simple_hardware.cpp,.hpp}`、`openarm_hardware.xml`、`openarm/CMakeLists.txt`
**你要做的**
1. 新建 `y7_hardware` 包，写自己的 `SystemInterface`，`read/write` 对接 vcan。
2. 新建**真机版** `y7_arm_lite.ros2_control.xacro`（`<plugin>y7_hardware/Y7System</plugin>`），保留 mock 版以便切回。
3. 把 URDF 里占位的 `<limit effort/velocity>` 换成电机+减速器真实值（P8）。
4. 开 `effort` 接口（为 D5 的重力补偿铺路）。
**验证命令**
```bash
ros2 launch y7_bringup bringup.launch.py use_fake_hardware:=false
ros2 control list_hardware_interfaces          # command/state 接口是否完整
ros2 control list_controllers -v
candump vcan0                                  # 肉眼确认报文在动
ros2 topic hz /joint_states
```
**完成标准**：RViz Plan&Execute → vcan 上看到对应报文 → `/joint_states` 跟动；`Ctrl-C` 后安全停下（`on_deactivate` 生效）。

---

### D11+ ⬜ 真机
**目标**：安全接入真实电机。
**顺序（不可颠倒）**
1. 静态标定：手动摆到零点 → 读编码器 → 建立**关节角 ↔ 电机位置**映射（方向符号、减速比、偏置）。
2. 单关节低速小幅度点动（每次只使能一个关节）。
3. 全关节低速限幅运行，`update_rate` 从低往高调（100 → 250 → 500）。
4. 加重力补偿（D5 产出），用 `/manipulator_controller/controller_state` 的 `error.positions` 量化效果。
5. 设急停与软件限位；录 bag 做回归。
**参考**：`openarm_simple_hardware.cpp` 的 `return_to_zero()`/`on_deactivate()`/`kp_`/`kd_` 增益表。
**完成标准**：连续 30 分钟运行无失控；误差指标有数据支撑。

---

## 5. 陷阱清单（Trap List）

> 每一条都是**实测出来的**，不是理论。踩到时回来查这里。

### T1 ⚠️ `install/` 不是真相源（本次最大的坑）
`colcon build --symlink-install` **只创建/更新链接，从不删除**。源文件删了，install 里的残留会以三种形态骗你：

| 形态 | 本次实例 | 危害 |
| --- | --- | --- |
| 真实文件副本（非链接） | `install/.../config/ompl_planning.yaml`、`sensors_3d.yaml`，日期 9/3 | 你以为"我不能删，删了就跑不起来"，其实它是旧世界的幽灵 |
| 整个包目录残留 | `install/arm_description/` | 一个已归档的包还在 install 里存在，`ros2 pkg list` 可能仍能看到 |
| 悬空符号链接 | 源文件删掉后链接还在 | `find` 仍能列出，`cat` 报 no such file |

**纪律**：
- 任何"配置改了但行为没变"，第一个动作是 `rm -rf build install log` 重编，而不是去猜。
- 判断文件真身：`ls -l install/<pkg>/share/<pkg>/...`（看是不是 `->` 链接）。
- 只改 src，永远不只改 install。

### T2 ⚠️ colcon 会递归扫描**工作空间根目录**（不只是 `src/`）
证据：把 `arm_description` 移到 `57500/archive/` 后，`colcon build` 依然输出 `Starting >>> arm_description`——因为 `archive/` 也在工作空间根下，且里面有 `package.xml`。
**解决**：在不想被构建的目录里放一个空的 `COLCON_IGNORE` 文件（`build/`、`install/`、`log/` 天生自带）。

### T3 ⚠️ URDF 根链接带惯性 → KDL 报错隐患
实测日志（`robot_state_publisher` 与 `move_group` 各报一次）：
```
[kdl_parser]: The root link base_link has an inertia specified in the URDF,
but KDL does not support a root link with an inertia.
As a workaround, you can add an extra dummy link to your URDF.
```
你的 `base_link` 在 URDF 里有 `<inertial>`（CAD 里基座质量 3.68 kg）。
**影响**：KDL 会**忽略**根链接惯性 → 现在只影响运动学精度/告警；到了**动力学阶段（重力补偿）会直接算错**。
**处理（二选一，D2 做）**：① 加一个无质量的 `dummy_link` 作为新根；② 把 `base_link` 的 `<inertial>` 去掉（基座固定不动，通常无害）。

### T4 ⚠️ 命令限幅默认是**关闭**的
实测日志：
```
[controller_manager]: Enforcing command limits is disabled. Command limits from URDF will be ignored.
```
意思是：控制器发出的超出关节限位的指令会**原样透传**到硬件接口。仿真里无所谓，**真机上是事故来源**。真机阶段必须确认这个参数与 `joint_limits.yaml` 的配合。

### T5 ℹ️ 有些 ERROR 是无害噪声
```
[move_group...occupancy_map_monitor]: No 3D sensor plugin(s) defined for octomap updates   ← ERROR
```
它来自 3D 感知/八叉树模块，而你没有 3D 传感器。**证据**：这条 ERROR 之后 move_group 依然打印了 `You can start planning now!`，控制器也全部激活。
**要学的判断力**：日志级别 ≠ 后果严重性。判断标准是"该模块的功能我是否需要"。

### T6 ℹ️ MoveIt Setup Assistant 的 6 个已知缺陷
见 `PROJECT_STATUS.md` §7，每次重新生成配置包后必须手修（joint_limits 整数、缺 ompl_planning、无效 sensors_3d、joints 为空、缺 robot_description 传递、缺 action_ns）。

### T7 ⚠️ 占位参数（真机前必须替换）
| 参数 | 当前值 | 问题 |
| --- | --- | --- |
| URDF `<limit velocity>` | 2（rad/s?） | CSV 导出表里该列为**空**，说明是手填/默认，非电机指标 |
| URDF `<limit effort>` | 50（N·m?） | 同上 |
| `joint_limits.yaml` `max_velocity/max_acceleration` | 2.0 / 1.0 | 拍脑袋值 |
| control layer / hardware 的 `kp`/`kd` | 未设置 | 真机伺服增益，openarm 用 `{70,70,70,60,10,10,10}` |

### T8 ⚠️ 轴向符号是标定阶段的头号杀手
你的 URDF 里 `joint5` 的轴是 `(0, 0, -1)`（**负 Z**），其余为 `(0,1,0)`/`(1,0,0)`/`(0,0,1)`。
真机标定要建立「关节角 ↔ 电机位置」映射时，**符号错了会反向猛冲**。D11 第 1 步必须先做符号与偏置。

### T9 ℹ️ `~/.bashrc` 里有失效的 source
每次开 shell 都报：
```
bash: /home/a57500/ros2_ws/install/setup.bash: 没有那个文件或目录
bash: /home/a57500/ros2_study/install/setup.bash: 没有那个文件或目录
```
不影响功能，但会污染终端输出，调试时容易看漏真正错误（建议清理或加 `[ -f ... ] &&` 判断）。

---

## 6. 变更日志（CHANGELOG）

### C1 — 2026-09-12｜D1 工程清理
| 项 | 内容 |
| --- | --- |
| **做了什么** | ① 归档 `arm_description`、两个 ROS1 launch、`export.log` 到 `archive/`，并加 `COLCON_IGNORE`；② 删 1.1G `.vscode/browse.vc.db` 与空 `textures/`；③ 全量清理 `build/ install/ log/` 后重建；④ 彻底移除 `ompl_planning.yaml`、`sensors_3d.yaml`（原只在 install 里）；⑤ 手动删 `install/arm_description/`；⑥ `git init` + `.gitignore` |
| **为什么** | 学习期最怕"状态不可复现、改坏不能回滚"；且 `install/` 里的幽灵文件会持续误导判断 |
| **怎么验证** | `du -sh .` → 1.3M（原 >1.1G）；`colcon build` → `Summary: 3 packages finished`；`ls install/.../config/` → 已无 ompl/sensors；Python 实测 `MoveItConfigsBuilder(...).to_moveit_configs()` → `pipelines=['ompl','pilz_industrial_motion_planner','chomp','stomp']`、`ompl planner_configs=24`（**证明 ompl_planning.yaml 不需要**）；`ros2 launch y7_moveit_config demo.launch.py use_rviz:=false` → 两个控制器 `Configured and activated`，move_group `You can start planning now!` |
| **影响** | 提交历史可回滚；`src/` 只剩 3 个包；**注意**：`ompl_planning.yaml` 现已完全不存在，若将来要自定义规划器需**新建**（见 §7 命令） |
| **遗留** | P1~P11（见 §2.4） |

### C2 — 2026-09-12｜git 工程化收尾（D0）
| 项 | 内容 |
| --- | --- |
| **做了什么** | ① 新增 `README.md`（项目说明/目录结构/编译启动/数据流/验证命令/工程纪律/Git 约定/待办）；② 新增 `.gitattributes`（文本统一 LF；`*.STL` 等标记为 binary，避免二进制被当文本 diff）；③ 打标签 `v0.1-simulation-closed-loop` |
| **为什么** | 建立"可回滚 + 可交接 + 有安全网"的基线；README 是新对话/新机器快速接管的第一入口 |
| **怎么验证** | `colcon build --symlink-install` → 3 packages finished；`git status --short` → 干净；`git tag -n` → `v0.1-simulation-closed-loop`；`git ls-files \| wc -l` → 56 |
| **影响** | 无功能变化；git 约定（`master` 常绿 + `feat/*` 实验 + 前缀式提交信息）从 D3 起执行 |
| **遗留** | 远端仓库未配置（需要地址后一条命令即可：`git remote add origin <url> && git push -u origin master --tags`） |

### 变更记录模板（以后照着填）
```
### C<N> — YYYY-MM-DD｜标题（对应 D<N>）
| 项 | 内容 |
| 做了什么 |  |
| 为什么   |  |
| 怎么验证 | 命令 + 期望输出 |
| 影响     | 影响哪些文件/包；是否需要重新生成配置包 |
| 遗留     | 新发现的问题（回填到 §2.4 或 §5） |
```

---

## 7. 参考地图（读哪个文件、学什么）

### 7.1 openarm（`/home/a57500/workspace/openarm/`）
| 文件 | 学什么 | 注意 |
| --- | --- | --- |
| `openarm_ros2/openarm_bringup/launch/openarm.bimanual.launch.py` | launch 工程化：xacro 生成 URDF、参数注入、TimerAction 顺序、spawner 寻址 | 精读，结构照搬，代码别抄 |
| `openarm_ros2/openarm_bringup/config/controllers/openarm_bimanual_controllers.yaml` | jtc 的完整参数语义（`interpolation_method: none` + `interpolate_from_desired_state` + `constraints`）、`update_rate: 750`、`forward_command_controller` 高频直发路线 | 这是"真机怎么配控制器"的教科书 |
| `openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp` | **阶段3 的主教材**：`on_init/on_configure/on_activate/read/write/return_to_zero`、状态与命令接口导出、gripper 的关节↔电机换算 | 读三遍：接口层 / 生命周期 / 安全动作 |
| `openarm_ros2/openarm_hardware/openarm_hardware.xml` | pluginlib 插件导出声明 | 写 `y7_hardware` 时照这个格式 |
| `openarm_can/src/openarm/damiao_motor/dm_motor_control.cpp` | 浮点→定点→位域打包；MIT 模式 | 只学**打包思路**，协议按你自己的驱动器 |
| `openarm_can/python/examples/*.py` | "最小可验证脚本"的写法（先单点验证硬件，再上框架） | 值得模仿的工程习惯 |

### 7.2 MoveIt 官方默认参数（本次的关键证据来源）
```
/opt/ros/jazzy/share/moveit_configs_utils/default_configs/
    ompl_planning.yaml     ← 你删掉的那份的"正版"，内容逐字节相同
    ompl_defaults.yaml     ← 24 个 OMPL 规划器默认参数（自动合并）
/opt/ros/jazzy/lib/python3.12/site-packages/moveit_configs_utils/moveit_configs_builder.py
    planning_pipelines()   ← 第 442-479 行：搜索+回退规则（建议你亲自读一遍）
    sensors_3d()           ← 第 407-422 行：文件不存在会被跳过；sensors 为空也不会加载
/opt/ros/jazzy/lib/python3.12/site-packages/moveit_configs_utils/launches.py
    generate_demo_launch() ← 第 340-360 行：就是你的 demo.launch.py 的真身（含 remap）
```
**将来若要自定义规划器**（比如只留 RRTConnect、加 RRTstar 参数），执行：
```bash
cp /opt/ros/jazzy/share/moveit_configs_utils/default_configs/ompl_planning.yaml \
   ~/workspace/openarm/57500/src/y7_moveit_config/config/
# 然后在 planner_configs / response_adapters 上做增删，并重编
```

### 7.3 你自己的 STM32 工程（`/home/a57500/workspace/openarm/MechanicalArm_Code_V4`）
| 文件 | 在 ROS2 学习中的用途 |
| --- | --- |
| `Cpp/Src/Trajectory_Planning.cpp` | 你的 S 曲线 + PID 中间层原型 → D5 要把它翻译成 ROS2 节点 |
| `Cpp/Src/MotorControl.cpp` | ZDT 步进驱动器的 CAN 帧格式 → D6/D8 打包的**真实依据** |
| `Cpp/Inc/RobotTypes.h` | DH 参数、Home 位姿、枚举状态机 → 对照 MoveIt 的 SRDF group_state / 控制器状态 |
| `Task/PTP_Task.cpp` | 定时器+队列驱动控制循环（100Hz）→ 对照 `update_rate` 与控制周期 |
| `Task/CppMain.cpp`+`FreeRTOSConfig.h` | 任务划分与优先级 → 对照 Executor / callback_group / RT 线程优先级 |
| `Cpp/Src/Inverse_Kinematics.cpp` | 931 行自研 IK → 对照 KDL 插件（**理解"为什么不必自己写"**） |

---

## 8. 开放问题（待确认，直接影响教学路线）

| # | 问题 | 为什么重要 | 谁来定 |
| --- | --- | --- | --- |
| **Q1** | **你这台 y7 机械臂最终用什么电机/驱动器？** STM32 项目用的是 ZDT 步进驱动器（CAN 模式，地址 1~6，减速比 50/51/27）；而 `y7_arm_lite` 是**7 关节**新结构。是继续用 ZDT？还是换达妙 DM？还是伺服？ | 决定 D6 报文格式、D8 插件里"关节角↔电机位置"的换算方式、以及是否有位置/力反馈 | 你 |
| **Q2** | **是否保留 MCU 做 1kHz 内环？**（PC → CAN 直连电机，还是 PC → MCU → 电机） | 决定中间层输出的是"高频位置点"还是"低频前馈+整段轨迹"；也决定 `update_rate` 定多少 | 你 |
| **Q3** | `sensors_3d.yaml` 是否需要恢复？（`sensors: []` 的无害版本可静音 T5 的部分噪声，但实测 ERROR 仍会出现，因为 move_group 的 occupancy monitor 与它无关） | 影响日志噪声，不影响功能 | 建议：**不恢复** |
| **Q4** | `~/.bashrc` 里两个失效 source（T9）是否清理？ | 影响调试体验 | 你 |
| **Q5** | 是否需要 `y7_arm_lite/urdf/y7_arm_lite.csv`？（CAD 导出的质量/惯量表，做动力学时是**宝贵的一手数据**） | 建议**保留**，并在 D5 用它核对 URDF 的 inertial | 建议：保留 |
| **Q6** | git 是否要加远端（GitHub）？ | 影响备份与多机同步 | 你 |

---

## 9. 立即行动（Next Actions）

1. **[你]** 回答 §8 的 Q1、Q2（决定后续所有 CAN 相关教学）。
2. **[你]** 按 D2 开始改 `trajectory_proxy.py` —— 我提供"改哪几处 + 怎么验证"，代码你写。
3. **[我]** D2 期间提供：Action 取消/抢占的最小验证脚本思路、`ros2 action info -t` 的读法、`cancel_callback` 的语义讲解。
4. **[你]** 每完成一步 → 在 §6 追加变更记录；发现问题 → 回填 §2.4 / §5。

---

## 10. 架构决策记录（ADR）

### ADR-1｜阻抗控制走"外部 MIT 律 + 驱动器电流环"（2026-09-12）
- **背景**：最终机械臂为 **6 轴**；驱动器确定为 **ZDT 步进驱动器**，**无 MIT 模式**，仅有 速度/位置/电流单环。
- **决策**：在外部（PC）实现 MIT 律，输出电流指令走驱动器的电流模式：
  `τ = Kp·(q*−q) + Kd·(q̇*−q̇) + τ_ff` → 换算电流(mA) → 电流模式帧
- **数学形式与 MIT 模式完全相同**，区别只是公式跑在哪里；驱动器内部电流环仍是"快内环"。
- **推论（核心）**：控制周期由 PC 决定 → **周期抖动与延迟成为第一性能指标**（不再是"插值好不好"）。
- **强制要求**：① 无位置内环 ⇒ **重力前馈必做**，否则下沉；② 步进无内部位置保护 ⇒ 必须有**位置误差超限保护**（失步检测）；③ URDF 的 inertial 必须可信（见 §5 T3）。
- **注意**：y7 是练习臂，最终 6 轴臂会重新导入 ⇒ 插件与中间层**不得硬编码 7 关节**（openarm 硬编码 `ARM_DOF=7` 并留了 TODO，别学这个坏习惯）。

### ADR-2｜先 USB-CAN 直连，MCU 版本延后（2026-09-12）
- **决策**：阶段3/4 先做 PC + USB-CAN 直连；MCU 作为后续可选加速项，**架构不变**。
- **前提（必须遵守）**：把「读状态 → 算力矩 → 发电流」封装成**与传输无关**的一层，将来整体搬进 MCU 的 1kHz 循环即可。
- **MCU 收益**：确定性周期（无 OS/GC/调度抖动）、1kHz 内环、PC 卡死时本地保护、多帧事务原子性。
- **MCU 成本**：双份代码 + 通信桥 + 状态同步 + 调试链路变长。
- **上 MCU 的量化判据（用数据决定，不靠感觉）**：① 实测周期抖动 > 5 ms；② 跟踪误差/振荡无法通过调参改善；③ **需要 > 200 Hz 的全轴反馈**（六轴定时返回 1ms 在 1Mbps 下占用约 91% 总线，PC+USB 无法支撑，只有 MCU 直连 CAN 能做）。满足任一条才上。

### ADR-3｜三种控制模式 ↔ 三种控制器（对应你 PadControl 的状态机）
| 模式 | ROS2 控制器 | 中间层职责 |
| --- | --- | --- |
| 位置/轨迹（MoveIt 规划执行） | `joint_trajectory_controller`（或 `interpolation_method: none` 透传） | 轨迹整形（S 曲线/限幅） |
| 阻抗/柔顺 | `forward_command_controller`（`Float64MultiArray` 高频下发力矩） | MIT 律 + 重力补偿 |
| 调试/单关节点动 | `forward_command_controller`（速度/电流） | 无 |

**模式切换 = `ros2 control switch_controllers`**（= 你 STM32 里 Standby/Cartesian/Joint/Tool 的切换）。

### ADR-4｜CAN 分包规范必须移植（2026-09-12）
ZDT 的位置模式命令共 **13 字节**（`addr, 0xFD, dir, vel(2), acc, clk(4), raF, syn, 0x6B`），超过经典 CAN 的 8 字节 ⇒ 厂商规范是**分包 + 扩展帧**：
`ExtId = (电机地址 << 8) | 包序号`，每帧首字节固定为功能码，其后最多 7 个数据字节。
- 证据：你的 `Core/Src/can.c:211 MyCAN1_Transmit()` 正是这个规范；STM32 侧实测波特率 `Prescaler=14, BS1=4TQ, BS2=1TQ` @APB1 42MHz ⇒ **500 kbps**。
- **后果**：Linux 侧（USB-CAN）必须**原样重新实现这套分包/组包**，否则位置模式根本发不出去；仅"能在 vcan 上发单个 8 字节帧"是不够的。

### ADR-5｜通信参数规范（帧预算，2026-09-12 定）
**适配器**：用户设备标有 canable / PCAN 标识且支持 Linux ⇒ 大概率是原生 SocketCAN（candleLight `gs_usb` 或 `peak_usb` 内核驱动）。
**待验证**：`lsusb`、`ip -br link | grep -i can`（若只有 `/dev/ttyUSB*` 则是 slcan，属降级方案）。
**波特率**：**1 Mbps**（与 STM32 侧 500 kbps 不同，这次是新链路，两边都要重配）。

**帧长基准（1 Mbps，扩展帧，含位填充的工程估算）**
| 帧 | 有效位数 | 耗时 |
| --- | --- | --- |
| 扩展帧 + 8 字节（定时返回） | 131 bit（+填充→≈140） | **≈140 μs** |
| 扩展帧 + 5 字节（电流指令） | 99 bit（+填充→≈110） | **≈110 μs** |

**场景预算（6 轴，速度用位置差分 ⇒ 只开位置返回）**
| 场景 | 帧/s | 总线占用 | 判定 |
| --- | --- | --- | --- |
| 单关节台架：返回 1ms + 外环 200Hz | 1200 | ≈16% | ✅ 轻松（可做到 500Hz 外环） |
| 六轴：返回 1ms + 外环 100Hz | 6600 | **≈91%** | ❌ 禁止（丢帧/重传，USB 侧也不堪重负） |
| 六轴：返回 5ms + 外环 100Hz | 1800 | ≈23% | ✅ **推荐** |
| 六轴：返回 2ms + 外环 200Hz | 4200 | ≈55% | ⚠️ 可行但吃紧 |

**规范（写死）**
1. **单关节台架**：定时返回 1ms 随便用。
2. **六轴**：定时返回 **5ms（200Hz 反馈）**，外环 100~200 Hz；**禁止**全轴 1ms。
3. **省带宽三招**（按需启用）：分组轮询（每周期只读半数轴、交替）、只开位置返回（速度差分）、关闭同步帧。
4. **天花板由反馈间隔决定**，不由 CAN 带宽决定；外环高于反馈频率无意义。

### ADR-6｜单关节带重物台架优先（2026-09-12）
- **背景**：最终 6 轴 URDF 未定；y7 只是练习臂。
- **决策**：在 6 轴 URDF 到位前，先在 **1 自由度台架**（电机 + 减速器 + 连杆 + 重物）跑通全链路，并顺便完成动力学参数标定。
- **阶段 A（脱离 ROS，python-can 脚本）**：A1 通信(1Mbps/扩展帧/分包) → A2 电流模式小电流验证 → A3 **Kt 标定**（恒定电流 + 弹簧秤测力）→ A4 **重力力矩 g(q) 标定**（各角度维持静止的最小电流，并与 URDF inertial 对比）→ A5 摩擦（可选）。
- **阶段 B（进 ROS，最小面积）**：B1 单关节最小 URDF → B2 插件只导 1 关节（state: position/velocity/effort，command: effort）→ B3 `forward_command_controller` + 30 行中间层（`τ = Kp·e + Kd·ė + τ_g(q)` @100Hz）→ B4 失步/超差保护 + 周期抖动测量。
- **里程碑**：**重力补偿下悬停 + 手推柔顺移动**。
- **明确的"不做"**：不为 y7 做精细标定；台架阶段**不牵扯 MoveIt/jtc/规划**（减少变量）。
- **性能预期（必须提前建立）**：外环可用带宽 ≈ 10~20 Hz 级的柔顺控制；**Kd 受 8~12 ms 等效延迟限制**（只能取临界阻尼的 1/3~1/5，需速度低通）；步进力矩脉动会让手感"偏糙"。做不了高刚度高动态力控。
- **安全纪律**：物理急停必须在手边；电流 0 → 100 → 300 mA 递增、每次只试 1 秒；重物先轻后重；必须有机械限位。

---

