# Y7 机械臂 ROS2 工程（57500）

从零手写的 ROS2 机械臂控制系统：目标是打通
`RViz/MoveIt 规划 → 中间控制层(整形/补偿) → ros2_control 硬件接口 → CAN → 电机`。

> **学习与规划文档（必读）**：[`ROS2_LEARNING_PLAN.md`](ROS2_LEARNING_PLAN.md)
> —— 里面有阶段计划（D1~D11）、陷阱清单（T1~T9）、架构决策记录（ADR-1~6）、变更日志。
> **技术笔记/踩坑记录**：[`PROJECT_STATUS.md`](PROJECT_STATUS.md)

## 当前状态

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 阶段1 | 仿真闭环：URDF + MoveIt + `mock_components` | ✅ 完成 |
| 阶段2 | 中间控制层：轨迹接收 / 整形 / 补偿 | 🚧 骨架完成（proxy 透传 + 监听器） |
| 阶段3 | 底层通信：CAN（`SystemInterface` 插件） | ⬜ 未开始 |
| 阶段4 | 真机：标定、限速、闭环验证 | ⬜ 未开始 |

## 环境

- ROS 2 **Jazzy**（`/opt/ros/jazzy`）
- 工作空间：`~/workspace/openarm/57500`
- 参考仓库（只读、不构建）：`../openarm_can`、`../openarm_description`、`../openarm_ros2`、`../MechanicalArm_Code_V4`

## 目录结构

```
57500/
├── src/
│   ├── y7_arm_lite/          # URDF 模型包（SolidWorks 导出）+ meshes
│   ├── y7_moveit_config/     # MoveIt 配置包（Setup Assistant 生成）+ ros2_control 配置
│   └── arm_control_layer/    # 中间控制层（Python 节点）
│       └── scripts/          # joint_state_listener / trajectory_proxy / ...
├── archive/                  # 历史归档（含 COLCON_IGNORE，不参与构建）
├── build/ install/ log/      # colcon 产物（.gitignore，不入库）
├── PROJECT_STATUS.md         # 技术笔记与踩坑记录
├── ROS2_LEARNING_PLAN.md     # 学习路线 / 阶段计划 / 变更日志 / ADR
└── README.md                 # 本文件
```

## 编译与启动

```bash
cd ~/workspace/openarm/57500
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**启动仿真（两个终端）**

```bash
# 终端 1：MoveIt + RViz + ros2_control
ros2 launch y7_moveit_config demo.launch.py

# 终端 2：中间控制层（必须启动，否则 RViz 里 Execute 会失败！）
ros2 run arm_control_layer trajectory_proxy
```

> ⚠️ `y7_moveit_config/config/moveit_controllers.yaml` 里的控制器名是 **`trajectory_proxy`**，
> 也就是 move_group 会把轨迹发给中间层。**不启动 proxy，Plan 能成功但 Execute 一定失败。**

## 当前数据流

```
RViz 拖动末端 → move_group 规划
   → (FollowJointTrajectory action) → trajectory_proxy（中间控制层）
   → (FollowJointTrajectory action) → manipulator_controller (joint_trajectory_controller)
   → mock_components/GenericSystem（仿真硬件）
   → /joint_states → robot_state_publisher / RViz
```

## 常用验证命令

```bash
ros2 control list_controllers                        # 控制器是否 active
ros2 action list | grep -E 'proxy|manipulator'       # action 接口是否存在
ros2 action info /trajectory_proxy/follow_joint_trajectory -t
ros2 topic hz /joint_states                          # 状态频率
ros2 topic info /joint_states --verbose              # QoS
ros2 param get /move_group moveit_simple_controller_manager.controller_names
```

## 工程纪律（血泪教训，务必遵守）

1. **`install/` 不是真相源**：`colcon build --symlink-install` 只创建/更新链接，**从不删除**。
   源文件删了，`install/` 里的残留会长期误导你（详见文档 T1）。
   → 任何"改了配置但行为没变"，**先 `rm -rf build install log` 重编**，不要靠猜。
2. **只改 `src/`**，永远不要只改 `install/` 里的文件。
3. **`archive/` 里有 `COLCON_IGNORE`**：colcon 会递归扫描工作空间根目录，不加这个标记归档目录照样会被构建（详见 T2）。
4. **MoveIt Setup Assistant 的 6 个已知缺陷**每次重新生成配置包后都要手修（详见 `PROJECT_STATUS.md` §7）。
5. **配置改坏不要慌**：`git checkout .` 回滚到最近一次提交。

## Git 约定

- 分支：`master` 始终保持**可用状态**；新功能/实验走 `feat/<名字>` 分支，失败直接丢弃。
- 提交信息前缀：`feat:` 新功能 / `fix:` 修 bug / `docs:` 文档 / `chore:` 清理与杂务 / `refactor:` 重构。
- **绝不入库**：`build/`、`install/`、`log/`、IDE 缓存、CAD 导出日志（已在 `.gitignore`）。
- 提交前自检：`git status --short`（确认没有意外的文件混进来）。

## 待办（优先级见 `ROS2_LEARNING_PLAN.md` §4 与 §9）

- [ ] D3：`y7_bringup` 一键启动（消灭"必须先手动起 proxy"的隐性依赖）
- [ ] D2：`trajectory_proxy` 加取消转发 / 抢占处理 / 改造回调链
- [ ] D4：`ros2 bag` + 周期抖动测量（决定将来是否上 MCU 的裁判工具）
- [ ] 补 `package.xml` 里的 `TODO: Package description` / `TODO: License declaration`
- [ ] 修 URDF 的 `base_link` 惯性（KDL 警告，动力学阶段会出问题，见 T3）
- [ ] 清理 `~/.bashrc` 里两个失效的 `source`（见 T9）
