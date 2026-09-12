# 项目进度总结

> **本文件作用**：记录当前项目的目标、已完成工作、遇到的问题及解决方案、学习路线。
> - 用于本人复习
> - 用于在更换对话时给新 AI 看，让其快速了解项目背景和当前状态
> - 最后更新时间：2026-09-04（阶段2进行中 — 架构理解完成）

---

## 一、项目总目标

从零构建 ROS2 系统，实现机械臂控制闭环：

```
RViz + MoveIt 规划 → 中间控制层（补偿/插值/实时控制） → CAN/MCU → 底层电机驱动器
```

### 核心要求
1. **模块解耦**：更换电机或通信协议时，只需修改 CAN 层
2. **手写实现**：不直接复用 OpenArm 代码，以学习为目的自己写
3. **参考资源**：OpenArm 的三个仓库克隆在 `/home/a57500/workspace/openarm/` 下，作为知识模板库参考
4. **MCU 可选**：如果有 MCU 会更好（PC 低频发送前馈数据，MCU 本地高频插值闭环）

### 用户背景
- 会写简单的 ROS 节点
- 理解话题、订阅者、服务等基础概念
- 学习目的，不照搬代码

---

## 二、整体学习路线（4 个阶段）

```
阶段1：仿真闭环                    ← ✅ 已完成
  RViz + MoveIt + 仿真控制器（mock_components）

阶段2：中间控制层                   ← ⏳ 进行中（架构理解完成，准备写轨迹监听节点）
  轨迹接收、插值补偿、实时控制逻辑
  已完成：joint_state_listener 节点 + ros2_control 架构理解
  下一步：trajectory_listener 节点（方案A：Python 节点拦截轨迹）

阶段3：底层通信层                   ← 之后
  CAN 通信 / MCU 通信（抽象 IActuatorInterface 接口）

阶段4：真实电机控制                 ← 最后
  替换 mock_components 为真实硬件接口
```

---

## 三、当前状态：阶段2 进行中

### 阶段1 已实现的功能
- [x] URDF 模型在 RViz 中正确显示
- [x] MoveIt 配置包生成并修复
- [x] ros2_control 仿真硬件接口（mock_components/GenericSystem）
- [x] 控制器配置（manipulator_controller + joint_state_broadcaster）
- [x] 运动学求解器（KDL）
- [x] 路径规划器（OMPL - RRTConnect）
- [x] RViz 中可拖动末端执行器交互标记
- [x] Plan & Execute 轨迹执行成功

### 阶段2 已实现的功能
- [x] 创建 arm_control_layer 包（中间控制层）
- [x] joint_state_listener 节点：订阅 /joint_states，打印关节角度和速度
- [x] 验证：Plan & Execute 时节点能正确收到关节角度变化
- [x] 理解 ros2_control 架构：controller_manager、控制器插件、硬件接口的关系
- [x] 理解 controller_manager 是系统固有二进制（类似 FreeRTOS 调度器），只配置不编辑
- [x] 理解控制器插件 vs 节点的区别（插件是 .so 被 manager 加载，节点是独立进程）
- [x] 理解 joint_trajectory_controller 和 joint_state_broadcaster 的分工（写指令 vs 读状态）
- [x] 确定方案A路线：先用 Python 节点拦截轨迹加补偿，后续再改方案B（C++ 插件）

### 阶段2 下一步
- [ ] trajectory_listener 节点：监听 MoveIt 发出的轨迹 action goal（方案A）

### 仿真闭环数据流
```
RViz 拖动末端 → MoveIt 规划路径 → joint_trajectory_controller
    → mock_components（仿真）→ /joint_states → RViz 显示运动
```

---

## 四、项目文件结构

```
/home/a57500/workspace/openarm/57500/          ← ROS2 工作空间
├── src/
│   ├── y7_arm_lite/                            ← URDF 模型包（SolidWorks 导出）
│   │   ├── urdf/y7_arm_lite.urdf               ← 机械臂模型（7关节）
│   │   ├── meshes/*.STL                        ← 3D 网格文件
│   │   └── launch/display.launch.py            ← URDF 显示用 launch
│   │
│   ├── y7_moveit_config/                       ← MoveIt 配置包
│   │   ├── config/
│   │   │   ├── y7_arm_lite.srdf                 ← 语义描述（规划组、末端执行器、碰撞矩阵）
│   │   │   ├── y7_arm_lite.ros2_control.xacro   ← ros2_control 硬件接口定义
│   │   │   ├── ros2_controllers.yaml            ← 控制器参数（给 controller_manager）
│   │   │   ├── moveit_controllers.yaml          ← MoveIt 控制器管理（给 move_group）
│   │   │   ├── joint_limits.yaml                ← 关节速度/加速度限制
│   │   │   ├── kinematics.yaml                  ← 运动学求解器配置（KDL）
│   │   │   ├── initial_positions.yaml           ← 初始关节位置
│   │   │   └── moveit.rviz                      ← RViz 配置
│   │   ├── launch/
│   │   │   ├── demo.launch.py                   ← 主启动文件（RViz + move_group + controller_manager）
│   │   │   └── ...
│   │   └── .setup_assistant                    ← Assistant 标记文件
│   │
│   └── arm_description/                         ← （早期尝试的包，可忽略）
│   │
│   └── arm_control_layer/                       ← 中间控制层包（阶段2新增）
│       ├── scripts/
│       │   └── joint_state_listener.py           ← 关节状态监听节点
│       ├── package.xml
│       └── CMakeLists.txt
│
├── build/                                       ← colcon 构建产物
├── install/                                     ← colcon 安装产物
└── PROJECT_STATUS.md                            ← 本文件

备份的旧配置（供参考）：
/home/a57500/workspace/openarm/y7_moveit_config/   ← 之前修复过的配置备份
```

---

## 五、关键配置文件说明

### 1. y7_arm_lite.srdf（语义描述）
- 定义了 2 个规划组：
  - `manipulator`：joint1~joint7（7个主动关节）
  - `end_effector`：只含 link7（标记末端位置）
- 末端执行器声明：`group=end_effector`, `parent_group=manipulator`（两者不能相同）
- 碰撞矩阵：Assistant 自动生成的 `<disable_collisions>` 列表
- home 姿态：所有关节为 0

### 2. ros2_controllers.yaml（给 controller_manager）
```yaml
controller_manager:
  manipulator_controller:    # 轨迹执行控制器
    type: joint_trajectory_controller/JointTrajectoryController
  joint_state_broadcaster:   # 状态广播器（发布 /joint_states）
    type: joint_state_broadcaster/JointStateBroadcaster

manipulator_controller:
  joints: [joint1~joint7]
  command_interfaces: [position]
  state_interfaces: [position, velocity]
```

### 3. moveit_controllers.yaml（给 move_group）
```yaml
moveit_simple_controller_manager:
  controller_names: [manipulator_controller]
  manipulator_controller:
    type: FollowJointTrajectory    ← 注意：这里是 action 类型简写
    action_ns: follow_joint_trajectory  ← ⚠️ 必须手动添加！Assistant 不生成此字段
    joints: [joint1~joint7]
```

**重要**：`action_ns` 告诉 move_group 去 `/<controller_name>/<action_ns>` 找 action server。
缺少此字段会导致 move_group 找不到控制器（返回 0 controllers），Plan 成功但 Execute 失败。

### 4. joint_limits.yaml（关节限制）
```yaml
joint1:
  has_velocity_limits: true
  max_velocity: 2.0              ← 必须是浮点数
  has_acceleration_limits: true   ← 必须开启，否则 AddTimeOptimalParameterization 报错
  max_acceleration: 1.0           ← 必须是浮点数且非0
```

### 5. y7_arm_lite.ros2_control.xacro（硬件接口）
- 插件：`mock_components/GenericSystem`（仿真用）
- 每个关节：command=position, state=position+velocity

---

## 六、遇到的问题及解决方案（按时间顺序）

### 问题1：MoveIt Setup Assistant 无法覆盖已有配置包
- **现象**：提示 "The chosen package location already exists but was not previously created using this MoveIt Setup Assistant"
- **原因**：目标目录缺少 `.setup_assistant` 标记文件
- **解决**：删除整个旧目录后重新生成

### 问题2：SRDF 末端执行器配置错误 "cannot be its own parent"
- **现象**：move_group 启动报错
- **原因**：在 Assistant 的 "Add End Effectors" 步骤里，把 End Effector Group 和 Parent Group 都选成了 `manipulator`
- **解决**：先在 "Define Planning Groups" 里定义 `end_effector` 组（只含 link7），再在末端执行器声明里设 `group=end_effector`, `parent_group=manipulator`

### 问题3：joint_limits.yaml 参数类型错误（致命，move_group 崩溃）
- **现象**：`InvalidParameterTypeException: expected [double] got [integer]`
- **原因**：Assistant 从 URDF 读取限制时，生成的是整数 `max_velocity: 2`，但 ROS2 要求浮点数
- **解决**：手动改成 `2.0`
- **注意**：这是 Assistant 的固定 bug，每次重新生成都会出现

### 问题4：ros2_controllers.yaml 的 joints 列表为空
- **现象**：控制器配置里 `joints: []`
- **原因**：在 Assistant 的 ROS2 Control 步骤里，添加控制器后没有做 "Add Components to Controller"
- **解决**：在 Assistant 里选中控制器 → Add Components → 勾选 joint1~joint7

### 问题5：ompl_planning.yaml 格式问题（Jazzy 版本变更）
- **现象**：`planning_plugins` 类型错误
- **原因**：Jazzy 版本要求字符串数组，旧版本是字符串
- **解决**：
  ```yaml
  # 错误（旧格式）
  planning_plugins: ompl_interface/OMPLPlanner
  # 正确（Jazzy 新格式）
  planning_plugins:
    - ompl_interface/OMPLPlanner
  ```
- **注意**：Assistant 不会生成这个文件，需手动创建

### 问题6：sensors_3d.yaml 引用不存在的插件
- **现象**：controller_manager 加载失败
- **原因**：Assistant 强制生成的默认传感器插件 `DepthImageOctomapUpdater` 系统里没有
- **解决**：删除 sensors_3d.yaml（没有 3D 传感器就不需要）

### 问题7：demo.launch.py 缺少 robot_description 参数
- **现象**：controller_manager 报 "Parameter not set"，不知道有哪些硬件接口
- **原因**：官方生成的 launch 文件没给 ros2_control_node 传 robot_description
- **解决**：在 demo.launch.py 里手动给 ros2_control_node 添加 `moveit_config.robot_description` 参数
- **注意**：本次重新生成后，demo.launch.py 用的是官方默认模板，可能需要再次检查

### 问题8：end_effector_controller 接口冲突
- **现象**：`Unable to activate controller 'end_effector_controller' since the command interface 'joint7/position' is currently claimed`
- **原因**：在 Assistant 里给 end_effector 组也配了一个 trajectory controller，它和 manipulator_controller 抢 joint7 的 position 接口
- **解决**：不要给 end_effector 组单独配控制器，末端执行器组只是标记，共享 manipulator 的控制器

### 问题9：AddTimeOptimalParameterization 失败（加速度限制缺失）
- **现象**：`No acceleration limit was defined for joint joint1!`
- **原因**：joint_limits.yaml 里 `has_acceleration_limits: false`，但 OMPL 的响应适配器需要加速度限制来计算时间参数
- **解决**：改成 `has_acceleration_limits: true`, `max_acceleration: 1.0`
- **注意**：URDF 的 `<limit>` 标签不支持 acceleration 属性，所以 Assistant 默认设为 false，必须手动开启

### 问题10：moveit_controllers.yaml 缺少 action_ns（Plan 成功但 Execute 失败）
- **现象**：`Returned 0 controllers in list`，`Unable to identify any set of controllers that can actuate the specified joints`
- **原因**：Assistant 生成的 moveit_controllers.yaml 里没有 `action_ns` 字段，MoveItSimpleControllerManager 不知道去哪找 action server
- **解决**：在 manipulator_controller 下添加 `action_ns: follow_joint_trajectory`
- **原理**：MoveItSimpleControllerManager 拼接 action 地址的规则是 `/<controller_name>/<action_ns>`，缺少 action_ns 就找不到 `/manipulator_controller/follow_joint_trajectory`
- **排查方法**：
  1. `ros2 control list_controllers` → 确认控制器在运行且 active
  2. `ros2 action list | grep manipulator` → 确认 action server 存在
  3. `ros2 action info /manipulator_controller/follow_joint_trajectory` → 看 action clients 是否为 0（为 0 说明 move_group 没连上）
  4. `ros2 param get /move_group moveit_simple_controller_manager.controller_names` → 确认参数加载正确
  5. 对比参数拼出的 action 地址和实际 action 地址是否一致

---

## 七、MoveIt Setup Assistant 的已知缺陷（Jazzy 版本）

以下问题**无法在 Assistant 里一次性解决**，每次重新生成后必须手动修复：

| # | 问题 | 手动修复方法 |
|---|------|-------------|
| 1 | joint_limits.yaml 生成整数类型 | max_velocity/max_acceleration 加小数点 |
| 2 | 不生成 ompl_planning.yaml | 手动创建，用数组格式 |
| 3 | 生成无效的 sensors_3d.yaml | 删除该文件 |
| 4 | ros2_controllers.yaml 可能 joints 为空 | 确认做了 Add Components |
| 5 | demo.launch.py 缺 robot_description 参数 | 手动给 ros2_control_node 传参 |
| 6 | moveit_controllers.yaml 缺少 action_ns | 手动添加 `action_ns: follow_joint_trajectory` |

### Assistant 里的正确操作流程
```
1. Start                          → 选 URDF
2. Self-Collisions                → 生成碰撞矩阵
3. Planning Groups                → 定义 manipulator 组 + end_effector 组
4. Robot Poses                    → 定义 home 姿态
5. Passive Joints                 → 跳过（没有被动关节）
6. ROS2 Control                   → 配置硬件接口 + 2个控制器 + Add Components
7. ROS2 Controllers (MoveIt)      → 配置 MoveIt 用哪个控制器
8. 3D Perception                  → 跳过
9. Author Information            → 填名字
10. Generate Package              → 生成后手动修复上述6个问题
```

---

## 八、两个容易混淆的概念

### 1. 两个 Controllers 配置的区别
| 文件 | 谁读它 | 作用 |
|------|--------|------|
| `moveit_controllers.yaml` | move_group | 告诉 MoveIt "轨迹发给哪个控制器" |
| `ros2_controllers.yaml` | controller_manager | 控制器本身的参数 |
| `y7_arm_lite.ros2_control.xacro` | controller_manager | 硬件接口定义 |

### 2. Controller Type 在两个步骤里的不同写法
| 在哪配 | Type 长什么样 |
|--------|--------------|
| ROS2 Control 步骤 | `joint_trajectory_controller/JointTrajectoryController`（插件全名） |
| ROS2 Controllers 步骤 | `FollowJointTrajectory`（action 类型简写） |

### 3. ROS2 三大通信机制
| 机制 | 特点 | 使用场景 |
|------|------|---------|
| **Topic** | 单向广播，发完就完 | 持续状态发布（如 /joint_states） |
| **Service** | 一问一答，阻塞等待 | 快速查询/设置（如获取参数） |
| **Action** | 一问多答，可取消，有反馈 | 耗时任务（如轨迹执行） |

Action 的消息结构 = Goal（请求）+ Feedback（中间反馈）+ Result（最终结果），用 `---` 分隔。

---

## 九、运行命令

```bash
# 进入工作空间
cd /home/a57500/workspace/openarm/57500

# 编译
colcon build --symlink-install

# source 环境
source install/setup.bash

# 启动 MoveIt 仿真
ros2 launch y7_moveit_config demo.launch.py
```

### 常用调试命令
```bash
# 检查控制器列表
ros2 control list_controllers

# 检查话题
ros2 topic list | grep joint_states

# 检查 move_group 参数
ros2 param get /move_group robot_description_semantic

# 检查 controller_manager 参数
ros2 param get /controller_manager robot_description

# 检查 action 接口（排查 Execute 失败时用）
ros2 action list | grep manipulator
ros2 action info /manipulator_controller/follow_joint_trajectory

# 检查 move_group 的控制器参数
ros2 param list /move_group | grep controller
ros2 param get /move_group moveit_simple_controller_manager.controller_names
```

---

## 十、阶段2 进度（中间控制层）

### 已完成
1. **创建 arm_control_layer 包**
   - 依赖：rclpy, sensor_msgs, trajectory_msgs, control_msgs
   - 位置：`src/arm_control_layer/`

2. **joint_state_listener 节点** ✅
   - 订阅 `/joint_states`（sensor_msgs/msg/JointState）
   - 打印关节名、角度、速度
   - 用时间间隔控制打印频率（避免 100Hz 刷屏）
   - 验证通过：Plan & Execute 时能正确收到关节角度变化

### 下一步
3. **trajectory_listener 节点**（监听 MoveIt 轨迹）
   - 需要理解 Action 通信（Goal/Feedback/Result）
   - 两种实现方式：
     - 方式A：偷听 action goal 话题（简单，只读不改）
     - 方式B：写 action client（可以拦截、修改、转发轨迹）
   - 先用方式A练习

### 之后
4. **插值补偿节点**
   - 把 MoveIt 的低频轨迹（~10Hz）插值成高频指令（~100Hz）
   - 加入重力补偿、摩擦补偿
   - 涉及动力学计算

5. **状态反馈节点**
   - 接收电机当前角度
   - 发布到 /joint_states 让 RViz 显示
   - 做闭环误差比较

### 学习收获
- ROS2 包和节点的关系：包是组织单位，节点是运行单位，一个包可含多个节点
- 依赖怎么确定：从话题反推 → `ros2 topic info` → 消息类型 `包名/msg/消息名` → 包名就是依赖
- ROS2 消息包命名规则：复数带 s（sensor_msgs, trajectory_msgs, control_msgs）
- Topic vs Action：/joint_states 是 Topic（实时帧），轨迹执行是 Action（完整路径+反馈）
- 控制器名 = 话题命名空间前缀（manipulator_controller → /manipulator_controller/...）
- 打印频率控制：用时间间隔而非取模，不受发布者频率影响
- **ros2_control 架构三层**：controller_manager（调度器）→ 控制器插件（.so）→ 硬件接口（read/write）
- **controller_manager**：系统固有二进制（`/opt/ros/jazzy/lib/controller_manager/ros2_control_node`），类似 FreeRTOS 调度器，只配置不编辑
- **控制器插件 vs 节点**：插件是 .so 被 manager 动态加载，在同一个进程里直接函数调用（零延迟）；节点是独立进程，通过话题通信（有延迟）
- **两个控制器的分工**：manipulator_controller 写 command_interface（控制），joint_state_broadcaster 读 state_interface（广播），缺一不可
- **joint_trajectory_controller**：ROS2 官方编译好的二进制（~1400-1500 行），项目里只配置了"要用它"，没有它的源码
- **方案A vs 方案B**：
  - 方案A：Python 节点拦截轨迹→加补偿→转发给官方控制器（有 1-5ms 延迟，学习阶段用）
  - 方案B：C++ 控制器插件替代官方控制器（零延迟，生产阶段用）
  - 先 A 后 B：核心算法一样，只是换个壳子
- **OpenArm 的实现方式**：官方 joint_trajectory_controller + 自己写的 CAN 硬件接口，没有写自定义控制器

### 设计原则
- 抽象 `IActuatorInterface` 接口（适配器模式）
- 换电机/通信协议时只改 CAN 层实现
- PC 到 MCU：50Hz 低频前馈数据
- MCU 本地：1kHz 高频插值闭环

---

## 十一、参考资源

- OpenArm 仓库（克隆在 `/home/a57500/workspace/openarm/`）：作为知识模板参考
- ROS2 Jazzy 版本
- MoveIt2 Setup Assistant
```