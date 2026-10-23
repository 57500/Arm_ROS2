#!/usr/bin/env python3
"""上位机桥接节点：ROS 2 <-> 浏览器。

分工：
    IK   -> MoveIt /compute_ik
    执行 -> MoveIt /move_action（目标用关节角，保证预览=执行）
    急停 -> 取消目标 + 停控制器 + 硬件组件切 inactive

实现要点：所有 ROS 服务/动作调用都用 call_async + asyncio 轮询等待，
不用 Client.call()（它会调用 spin_until_future_complete，和外部 executor 冲突，
把事件循环拖慢到十几 Hz）。
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
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotState as MoveItRobotState,
)
from moveit_msgs.srv import GetPositionIK
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DEFAULT_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
IK_TIMEOUT_SEC = 0.05


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

        self.q = [0.0] * len(self.joint_names)
        self.dq = [0.0] * len(self.joint_names)
        self.stamp = 0.0
        self.urdf = ""
        self.urdf_received = False
        self.status = {"controllers": [], "hardware": []}
        self.component_name = "FakeSystem"
        self.move_goal_handle = None

        cbg = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10,
                                 callback_group=cbg)
        desc_qos = QoSProfile(depth=1,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self._on_urdf, desc_qos,
                                 callback_group=cbg)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=cbg)
        self.list_controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers", callback_group=cbg)
        self.list_hardware = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components",
            callback_group=cbg)
        self.set_hardware_state = self.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state",
            callback_group=cbg)
        self.switch_controller = self.create_client(
            SwitchController, "/controller_manager/switch_controller", callback_group=cbg)
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=cbg)
        self.move_group_params = self.create_client(
            GetParameters, "/move_group/get_parameters", callback_group=cbg)

        self.ws_clients = set()
        self.state_slots = {}   # ws -> {"payload":..., "event": Event}，只保留最新一帧
        self.loop = None

        self.get_logger().info(
            "桥接节点启动：group=%s tip=%s 关节=%s"
            % (self.group_name, self.tip_link, ",".join(self.joint_names)))

    # ---------------- ROS 回调 ----------------
    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.q[self.joint_names.index(name)] = float(pos)
        for name, vel in zip(msg.name, msg.velocity):
            if name in self.joint_names:
                self.dq[self.joint_names.index(name)] = float(vel)
        self.stamp = time.time()
        # 直接用数据到达驱动推送：100 Hz 的 /joint_states 就有 100 Hz 的 state，
        # 不再依赖定时器（定时器会被事件循环的抖动拖慢）。
        if self.loop is not None and self.state_slots:
            payload = {"type": "state", "q": list(self.q), "dq": list(self.dq),
                       "t": self.stamp}
            self.loop.call_soon_threadsafe(self.publish_state, payload)

    def _on_urdf(self, msg: String):
        self.urdf = msg.data
        self.urdf_received = True
        if self.loop is not None:
            asyncio.run_coroutine_threadsafe(
                self.broadcast({"type": "urdf", "urdf": msg.data}), self.loop)

    def snapshot(self):
        return list(self.q), list(self.dq), self.urdf, self.stamp

    # ---------------- async 调用辅助 ----------------
    async def wait_service(self, client, timeout=0.3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if client.service_is_ready():
                return True
            await asyncio.sleep(0.01)
        return client.service_is_ready()

    async def call_service(self, client, request, timeout=1.0):
        if not await self.wait_service(client):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            await asyncio.sleep(0.005)
        return future.result()

    async def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            await asyncio.sleep(0.01)
        return future.result()

    # ---------------- IK ----------------
    async def compute_ik(self, pos, quat, seed):
        if not await self.wait_service(self.ik_client, 0.3):
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

        resp = await self.call_service(self.ik_client, req, 1.0)
        if resp is None:
            return {"ok": False, "error": "IK 超时"}
        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            return {"ok": False, "error": "IK 无解 (code=%d)" % resp.error_code.val}

        sol = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        if any(name not in sol for name in self.joint_names):
            return {"ok": False, "error": "IK 结果缺少关节"}
        return {"ok": True, "q": [float(sol[name]) for name in self.joint_names]}

    # ---------------- 执行 ----------------
    async def execute(self, q, vel_scale, acc_scale):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline and not self.move_client.server_is_ready():
            await asyncio.sleep(0.05)
        if not self.move_client.server_is_ready():
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

        handle = await self.wait_future(self.move_client.send_goal_async(goal), 5.0)
        if handle is None:
            return {"ok": False, "error": "发送目标超时"}
        if not handle.accepted:
            return {"ok": False, "error": "MoveIt 拒绝了目标"}

        self.move_goal_handle = handle
        result = await self.wait_future(handle.get_result_async(), 60.0)
        self.move_goal_handle = None
        if result is None:
            return {"ok": False, "error": "执行超时（60 s）"}

        code = result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            return {"ok": False, "error": "MoveIt 返回错误码 %s" % code}
        return {"ok": True}

    # ---------------- 急停 ----------------
    async def estop(self):
        notes = []

        handle = self.move_goal_handle
        if handle is not None:
            handle.cancel_goal_async()
            notes.append("已取消 MoveIt 目标")

        req = SwitchController.Request()
        req.deactivate_controllers = [self.controller_name]
        req.strictness = SwitchController.Request.BEST_EFFORT
        if await self.call_service(self.switch_controller, req, 1.0) is not None:
            notes.append("已停控制器")

        req2 = SetHardwareComponentState.Request()
        req2.name = self.component_name
        req2.target_state = LifecycleState(id=LifecycleState.PRIMARY_STATE_INACTIVE)
        resp = await self.call_service(self.set_hardware_state, req2, 2.0)
        if resp is not None and resp.ok:
            notes.append("硬件已切 inactive（电机 0 速度）")
        else:
            notes.append("硬件切换失败")

        return {"ok": True, "detail": "; ".join(notes) if notes else "没有可执行的动作"}

    # ---------------- 状态 ----------------
    async def fetch_urdf(self):
        """没有 robot_state_publisher 时，从 move_group 参数取 URDF。"""
        if self.urdf_received:
            return
        req = GetParameters.Request()
        req.names = ["robot_description"]
        resp = await self.call_service(self.move_group_params, req, 1.0)
        if resp is not None and resp.values and resp.values[0].string_value:
            self.urdf = resp.values[0].string_value
            self.urdf_received = True
            self.get_logger().info("已从 move_group 参数读取 robot_description")
            await self.broadcast({"type": "urdf", "urdf": self.urdf})

    async def poll_status(self):
        controllers = []
        resp = await self.call_service(self.list_controllers, ListControllers.Request(), 0.5)
        if resp is not None:
            controllers = [{"name": c.name, "state": c.state} for c in resp.controller]

        hardware = []
        resp = await self.call_service(self.list_hardware, ListHardwareComponents.Request(), 0.5)
        if resp is not None:
            for comp in resp.component:
                hardware.append({"name": comp.name, "state": comp.state.label})
                self.component_name = comp.name

        self.status = {"controllers": controllers, "hardware": hardware}
        return self.status

    # ---------------- 推送 ----------------
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
            return_exceptions=True)

    async def _safe_send(self, websocket, text):
        try:
            await websocket.send(text)
        except Exception:  # noqa: BLE001
            self.ws_clients.discard(websocket)

    def publish_state(self, payload):
        """把最新一帧放进每个客户端的槽位；不等待网络。"""
        for slot in list(self.state_slots.values()):
            slot["payload"] = payload
            slot["event"].set()

    async def _state_sender(self, websocket, slot):
        """每个连接一个发送任务；客户端慢时只丢中间帧。"""
        try:
            while True:
                await slot["event"].wait()
                slot["event"].clear()
                payload = slot["payload"]
                if payload is None:
                    continue
                await websocket.send(json.dumps(payload, separators=(",", ":")))
        except asyncio.CancelledError:
            pass
        except Exception:  # noqa: BLE001
            pass

    async def status_loop(self):
        while True:
            try:
                await self.fetch_urdf()
                status = await self.poll_status()
                await self.broadcast({"type": "status", **status})
            except Exception as exc:  # noqa: BLE001
                await self.broadcast({"type": "log", "level": "error",
                                      "msg": "状态轮询失败: %s" % exc})
            await asyncio.sleep(1.0)

    async def ws_handler(self, websocket, *args):
        self.ws_clients.add(websocket)
        slot = {"payload": None, "event": asyncio.Event()}
        self.state_slots[websocket] = slot
        sender = asyncio.create_task(self._state_sender(websocket, slot))

        q, dq, urdf, _ = self.snapshot()
        await self.send_json(websocket, {
            "type": "hello",
            "joint_names": self.joint_names,
            "group": self.group_name,
            "tip_link": self.tip_link,
            "base_frame": self.base_frame,
            "mesh_package": self.mesh_package,
            "urdf": urdf,
            "q": q,
        })

        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except Exception:  # noqa: BLE001
                    continue
                await self.handle_client_message(websocket, msg)
        finally:
            sender.cancel()
            self.state_slots.pop(websocket, None)
            self.ws_clients.discard(websocket)

    async def handle_client_message(self, websocket, msg):
        kind = msg.get("type")

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
                await self.send_json(websocket, {"type": "ik", "seq": seq,
                                                 "ok": False, "error": "pos 不合法"})
                return
            if not (isinstance(quat, list) and len(quat) == 4):
                await self.send_json(websocket, {"type": "ik", "seq": seq,
                                                 "ok": False, "error": "quat 不合法"})
                return
            seed = msg.get("seed")
            if not (isinstance(seed, list) and len(seed) == len(self.joint_names)):
                seed, _, _, _ = self.snapshot()
            result = await self.compute_ik(pos, quat, seed)
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
            result = await self.execute(q, vel, acc)
            await self.broadcast({"type": "execute", **result})
            return

        if kind == "estop":
            await self.broadcast({"type": "log", "level": "warn", "msg": "收到急停命令"})
            result = await self.estop()
            await self.broadcast({"type": "estop", **result})
            return


def make_http_handler(web_dir, ws_port):
    """静态文件服务；/robot/<pkg>/... 映射到对应包的 share 目录（给 STL 网格用）。"""

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=web_dir, **kwargs)

        def do_GET(self):
            # 前端用它确定 WebSocket 端口
            if self.path.startswith("/config.json"):
                body = json.dumps({"ws_port": ws_port}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            super().do_GET()

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

        def log_message(self, fmt, *args):   # 不刷屏
            return

    return Handler


async def run():
    bridge = Bridge()

    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    threading.Thread(target=executor.spin, daemon=True).start()

    bridge.loop = asyncio.get_running_loop()

    web_dir = os.path.join(get_package_share_directory("if_arm_web"), "web")
    httpd = ThreadingHTTPServer(
        (bridge.host, bridge.http_port), make_http_handler(web_dir, bridge.ws_port))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    async with websockets.serve(bridge.ws_handler, bridge.host, bridge.ws_port,
                                max_size=16 * 1024 * 1024):
        bridge.get_logger().info(
            "浏览器打开 http://%s:%d/  (WebSocket %d)"
            % (bridge.host, bridge.http_port, bridge.ws_port))
        await bridge.status_loop()


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



