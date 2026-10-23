#!/usr/bin/env python3
"""上位机桥接节点：ROS 2 <-> 浏览器。

分工（重要）：
    这个节点不做 IK、不规划轨迹、不限速。
    IK   -> 调 MoveIt 的 /compute_ik（和 RViz 拖拽用的是同一个求解器）
    执行 -> 把关节角作为 JointConstraint 发给 MoveIt 的 /move_action
    急停 -> 取消目标 + 停控制器 + 把硬件组件切到 inactive（触发 0 速度 + 关 CAN）

它只负责：采集状态、按 50 Hz 推给浏览器、把用户操作翻译成 MoveIt 请求。
"""

import asyncio
import json
import os
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

import rclpy
import websockets
from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.msg import State as LifecycleState
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState as MoveItRobotState,
)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
IK_TIMEOUT_SEC = 0.05          # 交给 MoveIt 的 IK 超时（和 RViz 同量级）
STATE_HZ = 50.0                # 推给浏览器的状态频率


class Bridge(Node):
    def __init__(self):
        super().__init__("if_arm_web_bridge")

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("http_port", 8000)
        self.declare_parameter("ws_port", 8001)
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("tip_link", "link6")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("controller_name", "manipulator_controller")
        self.declare_parameter("joint_names", DEFAULT_JOINTS)
        self.declare_parameter("mesh_package", "if_arm_lite")
        self.declare_parameter("ik_avoid_collisions", False)

        self.host = self.get_parameter("host").value
        self.http_port = int(self.get_parameter("http_port").value)
        self.ws_port = int(self.get_parameter("ws_port").value)
        self.group_name = self.get_parameter("group_name").value
        self.tip_link = self.get_parameter("tip_link").value
        self.base_frame = self.get_parameter("base_frame").value
        self.controller_name = self.get_parameter("controller_name").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.mesh_package = self.get_parameter("mesh_package").value
        self.ik_avoid_collisions = bool(self.get_parameter("ik_avoid_collisions").value)

        self._lock = threading.Lock()
        self.q = [0.0] * len(self.joint_names)
        self.dq = [0.0] * len(self.joint_names)
        self.stamp = 0.0
        self.urdf = ""
        self.urdf_received = False
        self.status = {"controllers": [], "hardware": [], "error": ""}
        self.component_name = "FakeSystem"     # 会被 list_hardware_components 覆盖
        self._move_goal_handle = None

        cbg = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10, callback_group=cbg
        )
        desc_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/robot_description", self._on_urdf, desc_qos, callback_group=cbg
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=cbg)
        self.list_controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=cbg
        )
        self.list_hardware = self.create_client(
            ListHardwareComponents,
            "/controller_manager/list_hardware_components",
            callback_group=cbg,
        )
        self.set_hardware_state = self.create_client(
            SetHardwareComponentState,
            "/controller_manager/set_hardware_component_state",
            callback_group=cbg,
        )
        self.switch_controller = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=cbg
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=cbg)
        self.get_move_group_params = self.create_client(
            GetParameters, "/move_group/get_parameters", callback_group=cbg
        )

        self.ws_clients = set()
        self.loop = None

        self.get_logger().info(
            "桥接节点启动：group=%s tip=%s 关节=%s"
            % (self.group_name, self.tip_link, ",".join(self.joint_names))
        )

    # ---------------- ROS 回调 ----------------
    def _on_joint_state(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self.joint_names:
                    i = self.joint_names.index(name)
                    self.q[i] = float(pos)
            for name, vel in zip(msg.name, msg.velocity):
                if name in self.joint_names:
                    i = self.joint_names.index(name)
                    self.dq[i] = float(vel)
            self.stamp = time.time()

    def _on_urdf(self, msg: String):
        with self._lock:
            self.urdf = msg.data
            self.urdf_received = True
        # URDF 晚到时也推给已连接的浏览器
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(
                self.broadcast({"type": "urdf", "urdf": msg.data}), self.loop
            )

    def snapshot(self):
        with self._lock:
            return list(self.q), list(self.dq), self.urdf, self.stamp

    # ---------------- 业务：IK / 执行 ----------------
    def compute_ik_blocking(self, pos, quat, seed):
        """调 MoveIt 的 IK。种子用当前关节角，和 RViz 拖拽一致。"""
        if not self.ik_client.wait_for_service(timeout_sec=0.2):
            return {"ok": False, "error": "compute_ik 不可用（move_group 没起？）"}

        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group_name
        r.ik_link_name = self.tip_link
        r.avoid_collisions = self.ik_avoid_collisions
        r.timeout = Duration(seconds=IK_TIMEOUT_SEC).to_msg()

        rs = MoveItRobotState()
        rs.is_diff = True
        js = JointState()
        js.name = list(self.joint_names)
        js.position = [float(v) for v in seed]
        rs.joint_state = js
        r.robot_state = rs

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(pos[0])
        ps.pose.position.y = float(pos[1])
        ps.pose.position.z = float(pos[2])
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])
        r.pose_stamped = ps

        try:
            resp = self.ik_client.call(req, timeout_sec=0.5)
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": "IK 调用异常: %s" % exc}

        if resp is None:
            return {"ok": False, "error": "IK 超时"}
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return {"ok": False, "error": "IK 无解 (code=%d)" % resp.error_code.val}

        sol = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        if any(name not in sol for name in self.joint_names):
            return {"ok": False, "error": "IK 结果缺少关节"}
        return {"ok": True, "q": [float(sol[name]) for name in self.joint_names]}

    def execute_blocking(self, q, vel_scale, acc_scale):
        """把关节角作为 JointConstraint 交给 MoveIt 规划并执行。"""
        if not self.move_client.wait_for_server(timeout_sec=0.5):
            return {"ok": False, "error": "/move_action 不可用"}

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.group_name
        req.num_planning_attempts = 10
        req.allowed_planning_time = 2.0
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)
        req.start_state.is_diff = True

        constraints = Constraints()
        for name, value in zip(self.joint_names, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        done = threading.Event()
        box = {}

        def on_result(future):
            try:
                box["code"] = future.result().result.error_code.val
            except Exception as exc:  # noqa: BLE001
                box["error"] = str(exc)
            done.set()

        def on_goal(future):
            try:
                handle = future.result()
            except Exception as exc:  # noqa: BLE001
                box["error"] = str(exc)
                done.set()
                return
            if not handle.accepted:
                box["error"] = "MoveIt 拒绝了目标"
                done.set()
                return
            self._move_goal_handle = handle
            handle.get_result_async().add_done_callback(on_result)

        self.move_client.send_goal_async(goal).add_done_callback(on_goal)

        if not done.wait(timeout=30.0):
            return {"ok": False, "error": "执行超时（30 s）"}
        self._move_goal_handle = None
        if "error" in box:
            return {"ok": False, "error": box["error"]}
        if box.get("code") != MoveItErrorCodes.SUCCESS:
            return {"ok": False, "error": "MoveIt 返回错误码 %s" % box.get("code")}
        return {"ok": True}

    # ---------------- 业务：急停 / 状态 ----------------
    def estop_blocking(self):
        """急停：取消目标 -> 停控制器 -> 硬件切 inactive（0 速度并关 CAN）。"""
        notes = []

        handle = self._move_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            notes.append("已取消 MoveIt 目标")

        if self.switch_controller.wait_for_service(timeout_sec=0.3):
            req = SwitchController.Request()
            req.deactivate_controllers = [self.controller_name]
            req.strictness = SwitchController.Request.BEST_EFFORT
            if self.switch_controller.call(req, timeout_sec=0.5) is not None:
                notes.append("已停控制器")

        if self.set_hardware_state.wait_for_service(timeout_sec=0.3):
            req = SetHardwareComponentState.Request()
            req.name = self.component_name
            req.target_state = LifecycleState(id=LifecycleState.PRIMARY_STATE_INACTIVE)
            resp = self.set_hardware_state.call(req, timeout_sec=1.0)
            if resp is not None and resp.ok:
                notes.append("硬件已切 inactive（电机 0 速度）")
            else:
                notes.append("硬件切换失败")

        return {"ok": True, "detail": "; ".join(notes) if notes else "没有可执行的动作"}

    def fetch_urdf_from_move_group(self):
        """没有 robot_state_publisher 时，直接从 move_group 的参数里取 URDF。"""
        if self.urdf_received:
            return
        if not self.get_move_group_params.wait_for_service(timeout_sec=0.2):
            return
        req = GetParameters.Request()
        req.names = ["robot_description"]
        resp = self.get_move_group_params.call(req, timeout_sec=1.0)
        if resp is not None and resp.values and resp.values[0].string_value:
            with self._lock:
                self.urdf = resp.values[0].string_value
                self.urdf_received = True
            self.get_logger().info("已从 move_group 参数读取 robot_description")
            if self.loop is not None:
                asyncio.run_coroutine_threadsafe(
                    self.broadcast({"type": "urdf", "urdf": self.urdf}), self.loop
                )

    def poll_status_blocking(self):
        self.fetch_urdf_from_move_group()
        controllers = []
        if self.list_controllers.wait_for_service(timeout_sec=0.2):
            resp = self.list_controllers.call(ListControllers.Request(), timeout_sec=0.5)
            if resp is not None:
                controllers = [{"name": c.name, "state": c.state} for c in resp.controller]

        hardware = []
        if self.list_hardware.wait_for_service(timeout_sec=0.2):
            resp = self.list_hardware.call(ListHardwareComponents.Request(), timeout_sec=0.5)
            if resp is not None:
                for comp in resp.component:
                    hardware.append({"name": comp.name, "state": comp.state.label})
                    self.component_name = comp.name   # 自动跟随实际组件名

        self.status = {"controllers": controllers, "hardware": hardware, "error": ""}
        return self.status

    # ---------------- 浏览器：推送 ----------------
    async def send_json(self, websocket, payload):
        try:
            await websocket.send(json.dumps(payload, separators=(",", ":")))
        except Exception:  # noqa: BLE001
            self.ws_clients.discard(websocket)

    async def broadcast(self, payload):
        if not self.ws_clients:
            return
        text = json.dumps(payload, separators=(",", ":"))
        await asyncio.gather(
            *[self._safe_send(ws, text) for ws in list(self.ws_clients)],
            return_exceptions=True,
        )

    async def _safe_send(self, websocket, text):
        try:
            await websocket.send(text)
        except Exception:  # noqa: BLE001
            self.ws_clients.discard(websocket)

    async def state_loop(self):
        """按 STATE_HZ 推关节状态（40~50 Hz 足够，前端再做插值）。"""
        interval = 1.0 / STATE_HZ
        while True:
            await asyncio.sleep(interval)
            q, dq, _, stamp = self.snapshot()
            await self.broadcast({"type": "state", "q": q, "dq": dq, "t": stamp})

    async def status_loop(self):
        loop = asyncio.get_running_loop()
        while True:
            try:
                status = await loop.run_in_executor(None, self.poll_status_blocking)
                await self.broadcast({"type": "status", **status})
            except Exception as exc:  # noqa: BLE001
                await self.broadcast({"type": "log", "level": "error", "msg": "状态轮询失败: %s" % exc})
            await asyncio.sleep(1.0)

    # ---------------- 浏览器：连接与命令 ----------------
    async def ws_handler(self, websocket, *args):
        self.ws_clients.add(websocket)
        q, dq, urdf, stamp = self.snapshot()
        await self.send_json(
            websocket,
            {
                "type": "hello",
                "joint_names": self.joint_names,
                "group": self.group_name,
                "tip_link": self.tip_link,
                "base_frame": self.base_frame,
                "mesh_package": self.mesh_package,
                "urdf": urdf,
                "q": q,
            },
        )
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except Exception:  # noqa: BLE001
                    continue
                await self.handle_client_message(websocket, msg)
        finally:
            self.ws_clients.discard(websocket)

    async def handle_client_message(self, websocket, msg):
        kind = msg.get("type")
        loop = asyncio.get_running_loop()

        if kind == "ping":
            await self.send_json(websocket, {"type": "pong", "t": time.time()})
            return

        if kind == "urdf":
            _, _, urdf, _ = self.snapshot()
            await self.send_json(websocket, {"type": "urdf", "urdf": urdf})
            return

        if kind == "ik":
            seq = msg.get("seq", 0)
            pos = msg.get("pos")
            quat = msg.get("quat")
            if not (isinstance(pos, list) and len(pos) == 3):
                await self.send_json(websocket, {"type": "ik", "seq": seq, "ok": False,
                                                 "error": "pos 不合法"})
                return
            if not (isinstance(quat, list) and len(quat) == 4):
                await self.send_json(websocket, {"type": "ik", "seq": seq, "ok": False,
                                                 "error": "quat 不合法"})
                return
            seed = msg.get("seed")
            if not (isinstance(seed, list) and len(seed) == len(self.joint_names)):
                seed, _, _, _ = self.snapshot()
            result = await loop.run_in_executor(None, self.compute_ik_blocking, pos, quat, seed)
            result.update({"type": "ik", "seq": seq})
            await self.send_json(websocket, result)
            return

        if kind == "execute":
            q = msg.get("q")
            if not (isinstance(q, list) and len(q) == len(self.joint_names)):
                await self.send_json(websocket, {"type": "log", "level": "error",
                                                 "msg": "目标关节角不合法"})
                return
            vel = max(0.01, min(1.0, float(msg.get("vel_scale", 0.2))))
            acc = max(0.01, min(1.0, float(msg.get("acc_scale", 0.2))))
            await self.broadcast({"type": "log", "level": "info", "msg": "开始规划并执行…"})
            result = await loop.run_in_executor(None, self.execute_blocking, q, vel, acc)
            await self.broadcast({"type": "execute", **result})
            return

        if kind == "estop":
            await self.broadcast({"type": "log", "level": "warn", "msg": "收到急停命令"})
            result = await loop.run_in_executor(None, self.estop_blocking)
            await self.broadcast({"type": "estop", **result})
            return


def make_http_handler(web_dir, ws_port):
    """静态文件服务；/robot/<pkg>/... 映射到对应包的 share 目录（给 STL 网格用）。"""

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

        def translate_path(self, path):
            parsed = urlparse(path)
            clean = unquote(parsed.path)
            if clean.startswith("/robot/"):
                parts = clean[len("/robot/"):].split("/", 1)
                if len(parts) == 2:
                    pkg, rel = parts
                    try:
                        base = get_package_share_directory(pkg)
                    except Exception:  # noqa: BLE001
                        base = None
                    if base:
                        return os.path.join(base, rel)
            return super().translate_path(path)

        def do_GET(self):
            # 前端用它确定 WebSocket 端口，端口改了页面不用改
            if self.path.startswith("/config.json"):
                body = json.dumps({"ws_port": ws_port}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

        def log_message(self, fmt, *args):   # 不刷屏
            return

    return Handler


async def run():
    bridge = Bridge()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    bridge.loop = asyncio.get_running_loop()

    web_dir = os.path.join(get_package_share_directory("if_arm_web"), "web")
    httpd = ThreadingHTTPServer(
        (bridge.host, bridge.http_port), make_http_handler(web_dir, bridge.ws_port)
    )
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    async with websockets.serve(
        bridge.ws_handler, bridge.host, bridge.ws_port, max_size=16 * 1024 * 1024
    ):
        bridge.get_logger().info(
            "浏览器打开 http://%s:%d/  (WebSocket %d)"
            % (bridge.host, bridge.http_port, bridge.ws_port)
        )
        await asyncio.gather(bridge.state_loop(), bridge.status_loop())


def main():
    rclpy.init()
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()



