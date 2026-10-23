#!/usr/bin/env bash
# 一键启动：机械臂本体（MoveIt + ros2_control）+ Web 上位机
#
# 用法：
#   ./start_hmi.sh              # 正常启动
#   ./start_hmi.sh --no-rviz    # 不打开 RViz
#   ./start_hmi.sh --force      # 先清掉已存在的控制器/move_group 再启动
#
# Ctrl+C 会依次发送 INT/TERM/KILL，确保两个 launch 和它们拉起的节点都退出。

# 注意：不能用 set -u —— ROS 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES
set +u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NO_RVIZ=0
FORCE=0
for arg in "$@"; do
  case "${arg}" in
    --no-rviz) NO_RVIZ=1 ;;
    --force)   FORCE=1 ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "找不到 ${WS_DIR}/install/setup.bash，请先 colcon build" >&2
  exit 1
fi

# ---- 启动前检查：避免重复启动导致 "more than one action server" ----
EXISTING="$(pgrep -f 'ros2_control_node|moveit_ros_move_group' | tr '\n' ' ')"
if [ -n "${EXISTING}" ]; then
  if [ "${FORCE}" = "1" ]; then
    echo "检测到已有节点（${EXISTING}），--force：先停掉它们…"
    pkill -INT -f 'ros2_control_node' 2>/dev/null
    pkill -INT -f 'moveit_ros_move_group' 2>/dev/null
    pkill -INT -f 'demo.launch.py' 2>/dev/null
    sleep 3
    pkill -9 -f 'ros2_control_node' 2>/dev/null
    pkill -9 -f 'moveit_ros_move_group' 2>/dev/null
    sleep 1
  else
    echo "已经有节点在运行：${EXISTING}" >&2
    echo "  - 先 Ctrl+C 停掉旧的 launch，或者" >&2
    echo "  - 用 ./start_hmi.sh --force 让脚本先清理" >&2
    exit 1
  fi
fi

WEB_PID=""
DEMO_PID=""

# 停一个进程：INT -> 等 5 秒 -> TERM -> 等 2 秒 -> KILL
stop_pid() {
  local pid="$1"
  [ -z "${pid}" ] && return
  kill -0 "${pid}" 2>/dev/null || return
  kill -INT "${pid}" 2>/dev/null
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || return
    sleep 0.25
  done
  kill -TERM "${pid}" 2>/dev/null
  for _ in $(seq 1 8); do
    kill -0 "${pid}" 2>/dev/null || return
    sleep 0.25
  done
  kill -KILL "${pid}" 2>/dev/null
}

cleanup() {
  echo ""
  echo "正在停止…"
  stop_pid "${WEB_PID}"
  stop_pid "${DEMO_PID}"

  # 兜底：清理 launch 没带走的子进程
  pkill -INT -f 'if_arm_web/lib/if_arm_web/bridge.py' 2>/dev/null
  pkill -INT -f 'ros2_control_node' 2>/dev/null
  pkill -INT -f 'moveit_ros_move_group' 2>/dev/null
  pkill -INT -f 'rviz2' 2>/dev/null
  sleep 2
  pkill -9 -f 'if_arm_web/lib/if_arm_web/bridge.py' 2>/dev/null
  pkill -9 -f 'ros2_control_node' 2>/dev/null
  pkill -9 -f 'moveit_ros_move_group' 2>/dev/null
  pkill -9 -f 'rviz2' 2>/dev/null
  echo "已停止"
  exit 0
}
trap cleanup INT TERM

echo "[1/3] 启动机械臂本体…"
if [ "${NO_RVIZ}" = "1" ]; then
  ros2 launch if_arm_lite_moveit_config demo.launch.py use_rviz:=false &
else
  ros2 launch if_arm_lite_moveit_config demo.launch.py &
fi
DEMO_PID=$!

echo "[2/3] 等待 controller_manager 就绪…"
READY=0
for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q "/controller_manager/list_controllers"; then
    READY=1
    break
  fi
  kill -0 "${DEMO_PID}" 2>/dev/null || break
  sleep 0.5
done
if [ "${READY}" = "0" ]; then
  echo "controller_manager 30 秒内没起来，看看本体那边的报错" >&2
fi

echo "[3/3] 启动 Web 上位机…"
ros2 launch if_arm_web web.launch.py &
WEB_PID=$!

echo ""
echo "=============================================="
echo "  浏览器打开:  http://127.0.0.1:8000/"
echo "  停止:        Ctrl+C"
echo "=============================================="
echo ""

# 任意一个退出就整体收尾
wait -n
cleanup
