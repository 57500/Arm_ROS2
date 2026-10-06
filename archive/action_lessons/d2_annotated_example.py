#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
#  【学习参考文件】不要运行、也不要加到 CMakeLists.txt 的 install(PROGRAMS)
#
#  文件名 : archive/action_lessons/d2_annotated_example.py
#  用途   : 把 ActionServer 的三件事 —— ①取消转发 ②goal 抢占 ③事件驱动等待 ——
#           用【假话题名 + 逐行注释】完整演示一遍。
#           你要写的是 src/arm_control_layer/scripts/trajectory_proxy.py，
#           结构照这里抄，话题名和业务逻辑换成你自己的。
#
#  阅读顺序（请不要从文件末尾的 main 开始看）：
#     ① 下面的「名词表」
#     ② GoalCtx 类         —— 一个 goal 的"档案袋"
#     ③ RelayDemo.__init__  —— 先看懂 4 个成员变量
#     ④ cancel_callback     —— 问题①：取消转发
#     ⑤ execute_callback    —— 问题②：抢占；问题③：不要忙等
#     ⑥ on_goal_response / on_result —— 谁负责把"红绿旗"升起来
# =============================================================================

# -----------------------------------------------------------------------------
# 【名词表】你问过的每个名字都在这。看清楚"它从哪来"——这决定了去哪查资料。
# -----------------------------------------------------------------------------
#
# A) Python 标准库（跟 ROS 无关，靠 import 拿进来，查 python 官方文档）
#    threading              Python 的多线程库
#    threading.Event        "红绿旗"：set() = 升旗；wait() = 一直等到旗升起
#    threading.Lock         "门锁"：同一时刻只允许一个线程拿钥匙进屋
#    bytes(x)               把一串整数变成"不可变的字节串"（这样才能当字典 key）
#    dict / {}              字典：键值对表，例如 {键1: 值1, 键2: 值2}
#    lambda f: 表达式        匿名函数（一个没名字的小函数）
#    with 语句              "离开这个块时自动收尾"，例如锁自动释放
#    None                   Python 的空值（相当于 C 里的 NULL）
#
# B) rclpy 提供（import rclpy / from rclpy.xxx import yyy，查 ROS2 文档）
#    Node                   节点基类，你的类都要继承它
#    ActionServer           动作服务器（你是"接单方"）
#    ActionClient           动作客户端（你是"下单方"，这里用来给下游下单）
#    CancelResponse         收到取消请求怎么回：ACCEPT=准 / REJECT=不准
#    GoalResponse           收到新目标接不接：ACCEPT / REJECT
#    ReentrantCallbackGroup 允许同一组的多个回调【并发】执行
#    MultiThreadedExecutor  多线程执行器（正因为有并发，才需要 threading.Lock）
#
# C) ROS 消息字段（是"数据"，不是函数；定义在 .msg 文件里，查消息定义）
#    goal_handle                   服务器手里"这一个 goal 的把手"。不是你创建的，
#                                  是 rclpy 创建并作为参数【传进】你的回调里的。
#    goal_handle.goal_id           这个 goal 的编号，类型是 unique_identifier_msgs/GoalID
#    goal_handle.goal_id.uuid      编号本体：16 个 0~255 的整数
#                                  （实测类型是 numpy.ndarray，所以不能直接当字典 key）
#    goal_handle.request           客户端发来的请求内容（FollowJointTrajectory.Goal）
#    goal_handle.is_active         True = 还没到终态（终态=SUCCEEDED/CANCELED/ABORTED）
#    goal_handle.is_cancel_requested  True = 有人申请取消，当前处于 CANCELING
#
# D) 命名习惯：cbg_ / ctx_ / lock_ 这种【下划线结尾】不是 ROS 规定，
#    是 ROS2 的 Python 代码风格约定 —— 成员变量加尾下划线，用来跟局部变量区分开。
#    看到 self.xxx_ 就知道是"这个类自己的成员变量"。
# -----------------------------------------------------------------------------

import threading          # A 类：Python 标准库

import rclpy              # B 类：ROS2 的 Python 客户端库
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory   # C 类：动作类型的定义


# =============================================================================
#  ② GoalCtx —— 一个 goal 的"档案袋"
# =============================================================================
class GoalCtx:
    """保存【一个 goal】在流转过程中要用到的全部信息。

    ctx 是什么？ = context 的缩写，"上下文/环境信息"。
        这个词在编程里很常见，意思是"处理这件事所需要的一整套相关数据"。
        觉得别扭的话你在自己代码里可以直接叫 GoalRecord（记录）或者 GoalState（状态）。

    为什么必须为每个 goal 单独建一个？
        因为回调是【并发】跑的（ReentrantCallbackGroup + MultiThreadedExecutor）：
        goal#1 还在跑的时候，goal#2 可能在另一个线程里就开始跑了。
        如果两个 goal 共用同一个 self.event_ 变量，goal#2 一升旗，
        goal#1 的 wait() 也会被唤醒 —— 两个 goal 就互相串门了。
        所以：一个 goal 一个档案袋，谁的事记在谁的袋子里，互不干扰。

    创建时机（你问的"在哪里创建的"）：
        每次有人下单、execute_callback 被调用时，第一行 new 一个：
            ctx = GoalCtx(goal_handle)
        也就是说：一个 goal 进来 → 一个档案袋 → 走到终态后从字典里删掉。
    """

    def __init__(self, upstream_gh):
        # 【红绿旗】每个 goal 单独一面。
        # 谁需要等结果，谁就 event.wait()；谁手里有结论了，谁就 event.set()。
        self.event = threading.Event()

        # 取消处理：
        #   本级的 goal_handle（服务器这边"这一个 goal 的把手"）。抢占时要标记旧 goal 的终态。
        self.upstream_gh = upstream_gh

        # 下游（真正的控制器）接受了我们的 goal 之后，返回的把手。
        # 取消时要靠它把 cancel 转发给下游。还没发出去之前是 None。
        self.downstream_gh = None

        # 下游最终回给我们的结果消息。还没回来时是 None。
        self.result = None

        # ★我们自己记的"有人要取消"。
        #   为什么不只看 rclpy 的 goal_handle.is_cancel_requested？
        #   因为 rclpy 是在 cancel_callback 【返回之后】才把状态置为 CANCELING 的
        #   （server.py:400-407）。如果下游反应极快，在这个窗口内就把结果发回来了，
        #   on_result 里 is_cancel_requested 还会是 False，于是被误判成"正常成功"。
        #   所以：谁最先知道"要取消"，谁就记在自己的档案袋里 —— 这就是那个"最先"。
        self.cancel_requested = False


# =============================================================================
#  ③ RelayDemo —— 代理节点本体
# =============================================================================
class RelayDemo(Node):
    """把上游来的 goal 转发给下游，并且正确处理 取消 / 抢占。"""

    def __init__(self):
        # 调用父类 Node 的构造函数，给节点起名字（ros2 node list 里显示的就是它）
        super().__init__('relay_demo')

        # ── 4 个成员变量，先看懂这 4 个，后面就顺了 ──────────────────────
        # 1) 回调组：告诉执行器"这一组回调允许【并发】执行"。
        #    为什么必须允许并发？因为 execute_callback 会阻塞（要等下游结果）。
        #    如果执行器只有一条线程、而且这组回调是"互斥"的，
        #    那么后面那些"通知结果"的回调就永远排不上队 → 死锁。
        self.cbg_ = ReentrantCallbackGroup()

        # 2) 档案柜：goal编号(bytes) → GoalCtx 对象。
        #    为什么不干脆用一个变量存"当前 goal"？
        #    因为抢占的那一瞬间，旧 goal 还在收摊、新 goal 已经开跑，
        #    同一时刻存在两个 goal，必须分开记，否则信息互相覆盖。
        self.ctx_ = {}

        # 3) 当前"正在跑"的那个 goal 的编号。新 goal 进来时，靠它找到上一个。
        self.active_id_ = None

        # 4) 门锁：self.ctx_ 和 self.active_id_ 会被多个线程同时读写，
        #    加锁保证"读出来→改一改→写回去"这一串动作中间不被打断。
        self.lock_ = threading.Lock()

        # ── 建 ActionServer（接单窗口）───────────────────────────────
        self.server_ = ActionServer(
            self,                                     # 挂在哪个节点上
            FollowJointTrajectory,                    # 动作类型
            'demo/upstream/follow_joint_trajectory',  # 假话题名（上游）
            execute_callback=self.execute_callback,   # 干活的钩子
            cancel_callback=self.cancel_callback,     # ★不写它 = 默认一律 REJECT
            callback_group=self.cbg_,                 # 放进允许并发的回调组
        )

        # ── 建 ActionClient（下单窗口，朝着下游）─────────────────────
        self.client_ = ActionClient(
            self,
            FollowJointTrajectory,
            'demo/downstream/follow_joint_trajectory',  # 假话题名（下游）
            callback_group=self.cbg_,
        )

    # ─────────────────────────────────────────────────────────────────────
    #  ④ cancel_callback —— 问题①：取消
    #     什么时候被调用：客户端发来取消请求时，由 rclpy 调用。
    #     它只回答一个问题：准不准取消？（顺便把取消转发给下游）
    #     注意：它【不负责】把 goal 变成终态，也【不许】在这里阻塞等待。
    # ─────────────────────────────────────────────────────────────────────
    def cancel_callback(self, goal_handle):
        # 第 1 步：从把手身上取出"这一个 goal 的编号"，转成 bytes 当字典的键。
        #     goal_handle.goal_id    → 类型是 unique_identifier_msgs/GoalID
        #     goal_handle.goal_id.uuid → 16 个 0~255 的整数（实测是 numpy.ndarray）
        #     为什么外面套一层 bytes()？
        #     numpy 数组是"可变序列"，Python 规定"可变对象不能当字典的 key"
        #     （会直接报 unhashable type）。bytes() 之后的字节串不可变，
        #     内容相同即相等，正好用来当 key 查找。这是 ROS2 处理 goal_id 的标准写法。
        gid = bytes(goal_handle.goal_id.uuid)

        # 第 2 步：加锁之后查档案柜。
        #     with self.lock_:  进这个块先拿钥匙，离开这个块自动还钥匙
        #                       （就算中间抛异常也会还，比手动 lock/unlock 安全）
        #     dict.get(键)      按键取值；键不存在时返回 None
        #                       （注意：写成 self.ctx_[gid] 键不存在会直接报错）
        with self.lock_:
            ctx = self.ctx_.get(gid)

        # 第 3 步：★先记下"有人要取消"这件事（写进这个 goal 自己的档案袋）。
        #     为什么要自己记：rclpy 是等你这个 cancel_callback 【返回之后】才把
        #     goal 状态置成 CANCELING 的；而下游如果反应极快，会在这个窗口内就把
        #     结果发回来（实测：日志打印花了 13ms，下游 2ms 就回结果了），
        #     那时 on_result 里 is_cancel_requested 还是 False，会被误判成"成功"。
        #     自己记 = 这个信息在所有回调里都可靠。
        if ctx is not None:
            ctx.cancel_requested = True

        # 第 4 步：打日志。★注意顺序：日志要放在"转发"之前！
        #     因为打日志本身要花十几毫秒，若放在转发之后，等于把
        #     "状态机还没更新"的那个竞态窗口撑大了十几倍。
        if ctx is not None and ctx.downstream_gh is not None:
            self.get_logger().warn('收到取消请求，准备转发给下游')

        # 第 5 步：★最后一步才是转发（真机安全的关键：不转发下游就会把轨迹走完）。
        #     放在最后 + 立刻返回，让 rclpy 尽快把状态推进到 CANCELING。
        if ctx is not None and ctx.downstream_gh is not None:
            ctx.downstream_gh.cancel_goal_async()

        # 第 6 步：表态。
        #     ACCEPT = 准（goal 状态会变成 CANCELING，但还不是终态）
        #     REJECT = 不准
        #     默认实现是 REJECT，所以"不写这个函数"= 取消永远被拒。
        return CancelResponse.ACCEPT


    # ─────────────────────────────────────────────────────────────────────
    #  ⑤ execute_callback —— 问题②：抢占  /  问题③：不要忙等
    #     谁调用它：rclpy 收到 goal 且被接受后，由执行器的线程调用（你不用自己调）。
    #     它必须返回一个 Result 消息；返回之前，goal 必须已经处于终态。
    # ─────────────────────────────────────────────────────────────────────
    def execute_callback(self, goal_handle):
        # ── 0) 建档：给这个 goal 建一个专属档案袋 ────────────────────────
        # bytes(...) 的原因同前面：字典的 key 必须是不可变对象。
        gid = bytes(goal_handle.goal_id.uuid)

        # 这里就是"每个 goal 对象的创建地点"：一个 goal 进来 = new 一个 GoalCtx。
        ctx = GoalCtx(goal_handle)

        # 把新 goal 登记进去，同时把"上一个在跑的 goal 编号"取出来。
        # 注意：进锁 → 读 old_id → 写 active_id_ → 出锁，这三步必须整体完成。
        with self.lock_:
            old_id = self.active_id_        # 先记住旧的（可能是 None）
            self.ctx_[gid] = ctx            # 新的登记进档案柜
            self.active_id_ = gid           # 更新"当前在跑的是谁"

        # ── 1) 抢占：有新 goal 来了，让旧的立刻收摊 ──────────────────────
        # 需要做三件事，少一件都会留下"僵尸 goal"：
        #   (a) 把旧 goal 的状态推入终态 CANCELED（否则它会以为自己还在跑）
        #   (b) 把取消转发给下游（否则电机还在按旧轨迹动！）
        #   (c) 升起旧 goal 的旗子（否则旧的 execute_callback 还在 wait() 里睡）
        if old_id is not None and old_id != gid:
            with self.lock_:
                # dict.pop(键, 默认值)：取出并删除；键不存在时返回默认值 None，不报错
                old_ctx = self.ctx_.pop(old_id, None)
            if old_ctx is not None:
                self.get_logger().warn('新目标抢占旧目标')
                # (a) 旧 goal 在本级进入终态。
                #     ★★ 这里必须是 abort()，不能是 canceled()！
                #     实测（rcl_action/goal_handle.c:100）：
                #       "invalid transition from state EXECUTING with event CANCELED"
                #     即 EXECUTING 不能直接跳 CANCELED（要先有人发 cancel 请求进 CANCELING）。
                #     抢占的语义本来也是"中止"，所以 ABORTED 是对的。
                if old_ctx.upstream_gh.is_active:
                    old_ctx.upstream_gh.abort()
                # (b) 通知下游别再走了
                if old_ctx.downstream_gh is not None:
                    old_ctx.downstream_gh.cancel_goal_async()
                # 被抢占的 goal 给个明确的失败码。不设的话默认 error_code=0，
                # 客户端会把它误读成"成功"（状态虽然已经是 CANCELED，但语义要清楚）。
                old_ctx.result = FollowJointTrajectory.Result()
                old_ctx.result.error_code = -1
                # (c) 升旗：唤醒旧 execute_callback 里那句 ctx.event.wait()
                old_ctx.event.set()

        # ── 2) 发给下游 ────────────────────────────────────────────────
        # wait_for_server(超时秒数)：等下游 ActionServer 出现。
        # 返回 True/False —— 所以你现在的代码里那句"等 10 秒再判断"可以整个删掉，
        # 换成这一行判断就够了，不需要自己写 while + sleep。
        if not self.client_.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('下游 Action Server 不存在，放弃本 goal')
            # 失败也要给"结果消息"：状态用 abort() 表达，业务码写在 result 里。
            # 为什么不能直接 return FollowJointTrajectory.Result()？
            #   因为该消息的 error_code 默认值是 0，而 0 恰好等于 SUCCESSFUL，
            #   只按 error_code 判断的客户端会把它误判成"成功"！所以要显式写 -1。
            result = FollowJointTrajectory.Result()
            result.error_code = -1
            # 不带参数（原因见 on_result 里的说明：状态和结果分工不同）
            goal_handle.abort()
            return result          # ★返回值才是客户端最终收到的 result

        # send_goal_async(...)：异步发送，立刻返回一个 Future（"期货合约"）。
        # Future = 一个"将来会有结果"的占位对象，它会在某时刻完成（done）。
        # add_done_callback(函数)：当它完成时，让执行器去调用这个函数，并把
        #   Future 本身作为参数传进去。这就是"回调链"——
        #   替代"while not future.done(): sleep(0.01)"的轮询。
        self.client_.send_goal_async(
            goal_handle.request,                      # 把上游的请求原样转发
            # ★feedback（执行过程中的中间进度）也要转发给上游，
            #   否则客户端（RViz / d2_test_client）看不到进度。
            #   它同样是"只接 1 个参数"的回调，所以也要用 lambda 包一层。
            feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback),
        ).add_done_callback(
            # lambda f: ... 是匿名函数，等价于：
            #     def 临时函数名(f): self.on_goal_response(f, goal_handle, ctx)
            # 为什么必须用 lambda？因为 add_done_callback 只会传进 Future 一个参数，
            # 而我们的处理函数还需要 goal_handle 和 ctx，只能在这里"包一层"传进去。
            lambda f: self.on_goal_response(f, goal_handle, ctx)
        )

        # ── 3) 等结论（问题③的核心：这里不是忙等）──────────────────────
        # event.wait() 会让当前线程【睡着，完全不占 CPU】，直到有人 event.set()。
        # 谁 set() 的？三个地方：
        #   · on_result        —— 下游出结果了（正常路径）
        #   · on_goal_response —— 下游拒绝了我们的 goal
        #   · 抢占分支          —— 被新 goal 顶掉了
        # 对比原来的写法：while not future.done(): time.sleep(0.01)
        #   time.sleep 也是让出 CPU 的（所以 CPU 不会飙升，计划里那句话不准确），
        #   但它是"每 10ms 醒一次看一眼"，既没有取消概念、又占着一条线程。
        # event.wait 是"睡到有人叫醒"，语义更清晰，且天然支持"被取消提前唤醒"。
        ctx.event.wait()

        # ── 4) 收摊与返回 ──────────────────────────────────────────────
        with self.lock_:
            if self.active_id_ == gid:      # 只有我还是"当前在跑的"才清空它
                self.active_id_ = None
            self.ctx_.pop(gid, None)        # 从档案柜注销

        # result 可能是 None（比如被抢占那条路径我们只给了 error_code），
        # 这时给一个空结果消息，保证返回类型正确。
        result = ctx.result if ctx.result is not None else FollowJointTrajectory.Result()

        # ★铁律：返回之前必须是终态。
        #   rclpy 的源码（server.py:367-371）里写着：如果 execute_callback 返回时
        #   goal 还没到终态，它会自动帮你 abort，并打印
        #   "Goal state not set, assuming aborted"。正常路径上这里不会被触发，
        #   它是兜底：防止你哪天加了个分支忘了设终态。
        if goal_handle.is_active:
            if goal_handle.is_cancel_requested:     # 判断"取消"看它，不要看 error_code
                goal_handle.canceled()              # 不带参数的写法，理由见 on_result
            else:
                goal_handle.abort()
        return result


    # ─────────────────────────────────────────────────────────────────────
    #  ⑥ 回调链 —— 谁负责升"红绿旗"
    # ─────────────────────────────────────────────────────────────────────
    def on_goal_response(self, future, goal_handle, ctx):
        """下游对我们的 goal 表态了（接受 / 拒绝）时被调用。

        参数从哪来：
          future      ← add_done_callback 自动传进来的那个 Future
          goal_handle ← 我们在 execute_callback 里用 lambda 绑进去的（本级的把手）
          ctx         ← 同上，是这个 goal 的档案袋
        """
        # future.result() = 取出 Future 的成果。此刻它已经完成，所以能立刻取到。
        # 这里拿到的是"下游客户端侧的 goal_handle"（类型 ClientGoalHandle）。
        downstream_gh = future.result()

        # .accepted = 下游同不同意接这单
        if not downstream_gh.accepted:
            self.get_logger().warn('下游拒绝了本 goal')
            ctx.result = FollowJointTrajectory.Result()
            ctx.result.error_code = -1
            ctx.event.set()          # ★必须升旗，否则 execute_callback 会一直睡
            return

        # 把下游把手登记进档案袋 —— 取消功能就靠它
        ctx.downstream_gh = downstream_gh

        # 再挂一跳：等下游的出最终结果
        downstream_gh.get_result_async().add_done_callback(
            lambda f: self.on_result(f, goal_handle, ctx)
        )

    def on_result(self, future, goal_handle, ctx):
        """下游给出最终结果时被调用。这里负责把本级的 goal 也收尾成终态。"""
        # GetResult 响应里有两个东西：
        #   .status  = 终态码（4=SUCCEEDED / 5=CANCELED / 6=ABORTED）
        #   .result  = 下游自己的 Result 消息（这里有 error_code）
        ctx.result = future.result().result

        # 三种结局，按【优先级】判断：
        # 1) 有人要取消 → 回 CANCELED。
        #    为什么先判取消，而不是先看 error_code？
        #    因为"取消"是【我们这边发生的事实】，由状态机记录，最可靠；
        #    而 error_code 是下游业务层自己填的，取消时它可能填 -1、也可能填别的。
        #    这里三个调用都【不带参数】，这是 ROS2 官方例子的标准写法，理由：
        #    客户端最终收到的 result，来自 execute_callback 的【返回值】。
        #    rclpy 源码 server.py:379 写得很明确：
        #        result_response.result = execute_result
        #    其中 execute_result 就是 execute_callback 返回的东西。
        #    所以把 result 传给 canceled(x)/succeed(x)/abort(x) 是【没意义】的，
        #    会被上面的返回值覆盖。统一约定：
        #        · 状态（SUCCEEDED/CANCELED/ABORTED） 由 xxx() 决定
        #        · 结果消息（error_code 等业务数据）  由 return 决定
        # ★★ 先判断 is_active！这是个很容易漏的坑：
        #    被抢占的 goal 我们早已 canceled() 掉（它是终态了），但我们转发给
        #    下游的那个 cancel 要过一会儿才生效，下游的结果随后照样会回来 ——
        #    于是这个回调【仍然会被调用】。此时再 abort() 就是"非法状态转换"
        #    （CANCELED → ABORTED），rclpy 会抛 RCLError。
        #    证据：rclpy server.py:405-410 的注释原话就是
        #    "If the goal's just succeeded after user cancel callback that will
        #     generate an exception from invalid transition."
        #    所以规矩是：任何设置终态之前，先问一句"我还没到终态吗"。
        if not goal_handle.is_active:
            ctx.event.set()      # 别忘了升旗，让 execute_callback 收工
            return

        # 是不是"取消"？【先看我们自己记的 flag，再看 rclpy 的状态】。
        #   只信 rclpy 会遇到竞态：cancel 被接受后它要等 cancel_callback 返回
        #   才置 CANCELING，而下游可能在这窗口内就把结果发回来了 —— 实测就是这样，
        #   结果被误判成"正常成功"（status=4 SUCCEEDED）。
        is_cancel = ctx.cancel_requested or goal_handle.is_cancel_requested

        if is_cancel and goal_handle.is_cancel_requested:
            goal_handle.canceled()      # 正常路径：CANCELING → CANCELED
        elif is_cancel:
            # 竞态兜底：状态机还没跟上（还是 EXECUTING），此时 canceled() 会抛
            # "invalid transition"，所以只能退化为 abort()。
            self.get_logger().warn('取消竞态兜底：状态机未及时进入 CANCELING，改用 abort()')
            goal_handle.abort()
        # 3) 下游说成功 → 本级成功
        elif ctx.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        # 4) 其它一律算失败
        else:
            goal_handle.abort()

        # ★无论哪种结局，最后都要升旗，把 execute_callback 放行。
        ctx.event.set()


# =============================================================================
#  ⑦ main —— 把节点转起来
# =============================================================================
def main():
    rclpy.init()                                   # 初始化 ROS2 运行时
    node = RelayDemo()                             # 创建节点（__init__ 里建好了 server/client）
    executor = MultiThreadedExecutor()             # 多线程执行器：回调可以并发
    rclpy.spin(node, executor=executor)            # 主线程在这里循环，直到 Ctrl-C
    node.destroy_node()                            # 清理节点
    rclpy.shutdown()                               # 关闭 ROS2 运行时


if __name__ == '__main__':
    main()


# =============================================================================
#  ⑧ 自测题（先自己答，答不出来再问）—— 答得出来就说明你真懂了
# =============================================================================
#  Q1. 如果 cancel_callback 里"忘了转发给下游"，实测会看到什么现象？
#      （提示：客户端会收到 CANCELED，但机械臂……）
#
#  Q2. 如果把 execute_callback 返回前那段"若还没进终态就补上 canceled()/abort()"
#      的兜底代码整个删掉，日志里会出现哪句 rclpy 的 warning？
#
#  Q3. 为什么 self.ctx_（档案柜）不能省掉、只用 self.active_goal_handle_ 一个变量？
#      （提示：抢占发生的那一瞬间，同时存在几个 goal？）
#
#  Q4. 如果把 ActionClient 的 callback_group 也换成"互斥组"
#      （MutuallyExclusiveCallbackGroup），会不会死锁？为什么？
#      （提示：execute_callback 正阻塞在 ctx.event.wait() 里，而 on_result
#        是"谁来调用"的？）
#
#  Q5. 换成你自己代码时，把下面的"假名 → 你的真名"对应关系写一遍：
#      demo/upstream/follow_joint_trajectory   →  ?
#      demo/downstream/follow_joint_trajectory →  ?
#      RelayDemo                               →  ?
# =============================================================================



