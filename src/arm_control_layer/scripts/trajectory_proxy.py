#!/usr/bin/env python3
import threading    

import rclpy          
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory  
from rclpy.executors import ExternalShutdownException

class GoalCtx:
    def __init__(self, upstream_gh):
        self.event=threading.Event()

        self.upstream_gh=upstream_gh
        self.downstream_gh=None

        self.cancel_requested = False

        self.result=None

class TrajectoryProxy(Node):
    def __init__(self):
        super().__init__('trajectory_proxy')

        self.cbg_=ReentrantCallbackGroup()

        self.ctx_={}

        self.active_id_=None

        self.lock_=threading.Lock()

        self.server_=ActionServer(
            self,
            FollowJointTrajectory,
            '/trajectory_proxy/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cbg_,
        )

        self.client_=ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory',
            callback_group=self.cbg_,
        )

    def cancel_callback(self,goal_handle):
        gid = bytes(goal_handle.goal_id.uuid)
        
        with self.lock_:
            ctx = self.ctx_.get(gid)

        if ctx is not None:
            ctx.cancel_requested = True

        self.get_logger().warn('收到取消请求')

        if ctx is not None and ctx.downstream_gh is not None:
            # 日志写在转发【之前】：打日志要花几~十几毫秒，
            # 放后面等于把"状态机还没进 CANCELING"的竞态窗口撑大。
            self.get_logger().warn('正在把取消转发给下游')
            ctx.downstream_gh.cancel_goal_async()

        return CancelResponse.ACCEPT

    def execute_callback(self,goal_handle):
        gid=bytes(goal_handle.goal_id.uuid)

        ctx=GoalCtx(goal_handle)

        with self.lock_:
            old_id=self.active_id_
            self.ctx_[gid]=ctx
            self.active_id_=gid

        if old_id is not None and old_id!=gid:
            with self.lock_:
                old_ctx=self.ctx_.pop(old_id,None)

            if old_ctx is not None:
                self.get_logger().warn('新目标抢占旧目标')

                if old_ctx.upstream_gh.is_active:
                    old_ctx.upstream_gh.abort()

                if old_ctx.downstream_gh is not None:
                    old_ctx.downstream_gh.cancel_goal_async()

                old_ctx.result = FollowJointTrajectory.Result()
                old_ctx.result.error_code = -1
                old_ctx.event.set()


        if not self.client_.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('下游 Action Server 不存在，放弃本目标')

            result=FollowJointTrajectory.Result()
            result.error_code=-1
            goal_handle.abort()
            return result

        # 取消来得早：还没发给下游就有人要取消 → 直接收摊。
        # 但别忘了"清账"：把档案删掉、把 active_id_ 清掉，
        # 否则 ctx_ 里会残留脏数据，active_id_ 还一直指着这个已经结束的 goal。
        if goal_handle.is_cancel_requested:
            with self.lock_:
                if self.active_id_ == gid:
                    self.active_id_ = None
                self.ctx_.pop(gid, None)
            result = FollowJointTrajectory.Result()
            result.error_code = -1      # 明确写 -1；默认 0 会被误读成 SUCCESSFUL
            goal_handle.canceled()
            return result

        self.client_.send_goal_async(
            goal_handle.request,
            feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback)
            ).add_done_callback(
            lambda future:self.on_goal_response(future,goal_handle,ctx)
        )

        ctx.event.wait()

        with self.lock_:
            if self.active_id_==gid:
                self.active_id_=None

            self.ctx_.pop(gid,None)

        result=ctx.result if ctx.result is not None else FollowJointTrajectory.Result()

        if goal_handle.is_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()

        return result

        

    def on_goal_response(self,future,goal_handle,ctx):
        downstream_gh=future.result()

        if not downstream_gh.accepted:
            self.get_logger().warn('下游拒绝了本 goal')

            ctx.result=FollowJointTrajectory.Result()
            ctx.result.error_code=-1
            ctx.event.set()

            return

        ctx.downstream_gh = downstream_gh
        if ctx.cancel_requested or goal_handle.is_cancel_requested:     
            downstream_gh.cancel_goal_async()

        downstream_gh.get_result_async().add_done_callback(
            lambda f:self.on_result(f,goal_handle,ctx)
        )

    def on_result(self,future,goal_handle,ctx):
        # 结果先记下来（即使这个 goal 已经是终态，也保留它最后的结果）
        ctx.result=future.result().result

        if not goal_handle.is_active: 
            ctx.event.set()
            return

        is_cancel = ctx.cancel_requested or goal_handle.is_cancel_requested

        if is_cancel and goal_handle.is_cancel_requested:
            goal_handle.canceled()   
        elif is_cancel:
            goal_handle.abort()        
        elif ctx.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        ctx.event.set()


def main():
    rclpy.init()
    node = TrajectoryProxy()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                  
    finally:
        node.destroy_node()
        if rclpy.ok():            
            rclpy.shutdown()                             


if __name__ == '__main__':
    main()