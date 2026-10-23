#!/usr/bin/env bash
# 一键启动：机械臂本体（MoveIt + ros2_control）+ Web 上位机
#
# 用法：
#   ./start_hmi.sh              # 正常启动
#   ./start_hmi.sh --no-rviz    # 不打开 RViz（只开本体 + 上位机）
#
# Ctrl+C 会同时停掉两个 launch。

# 注意：不能用 set -u —— ROS 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES
set +u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NO_RVIZ=0
[ "${1:-}" = "--no-rviz" ] && NO_RVIZ=1

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
else
  echo "找不到 ${WS_DIR}/install/setup.bash，请先 colcon build" >&2
  exit 1
fi

DEMO_PID=""
WEB_PID=""

cleanup() {
  echo ""
  echo "正在停止…"
  if [ -n "${WEB_PID}" ]; then kill -INT "${WEB_PID}" 2>/dev/null; fi
  if [ -n "${DEMO_PID}" ]; then kill -INT "${DEMO_PID}" 2>/dev/null; fi
  wait 2>/dev/null || true
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

wait "${WEB_PID}"
