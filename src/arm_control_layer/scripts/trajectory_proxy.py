#!/usr/bin/env python3
# import rclpy
# import time
# import threading
# from rclpy.node import Node
# from rclpy.action import ActionServer, ActionClient
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from control_msgs.action import FollowJointTrajectory


# class TrajectoryProxy(Node):
#     def __init__(self):
#         super().__init__('trajectory_proxy')

#         self.cb_group_ = ReentrantCallbackGroup()

#         self.action_server_ = ActionServer(
#             self,
#             FollowJointTrajectory,
#             '/trajectory_proxy/follow_joint_trajectory',
#             execute_callback=self.execute_callback,
#             callback_group=self.cb_group_
#         )

#         self.action_client_ = ActionClient(
#             self,
#             FollowJointTrajectory,
#             '/manipulator_controller/follow_joint_trajectory',
#             callback_group=self.cb_group_
#         )

#         self.get_logger().info('TrajectoryProxy 已启动')
#         self.get_logger().info('Action Server: /trajectory_proxy/follow_joint_trajectory')
#         self.get_logger().info('Action Client 目标: /manipulator_controller/follow_joint_trajectory')

#         thread = threading.Thread(target=self._wait_for_controller, daemon=True)
#         thread.start()

#     def _wait_for_controller(self):
#         self.get_logger().info('后台等待控制器 Action Server...')
#         self.action_client_.wait_for_server()
#         self.get_logger().info('控制器 Action Server 已就绪！')

#     def execute_callback(self, goal_handle):
#         goal = goal_handle.request
#         self.get_logger().info(f'[1] 收到轨迹：{len(goal.trajectory.points)} 个路径点')

#         if not self.action_client_.server_is_ready():
#             self.get_logger().warn('[2] 控制器未就绪，等待中（最多10秒）...')
#             ready = self.action_client_.wait_for_server(timeout_sec=10.0)
#             if not ready:
#                 self.get_logger().error('[2] 等待超时！控制器 Action Server 不存在')
#                 self.get_logger().error('请确认仿真已启动且 manipulator_controller 已激活')
#                 self.get_logger().error('运行 ros2 action list 检查')
#                 goal_handle.abort()
#                 result = FollowJointTrajectory.Result()
#                 result.error_code = -1
#                 return result

#         self.get_logger().info('[3] 控制器已就绪，发送 goal')

#         def on_feedback(feedback_msg):
#             goal_handle.publish_feedback(feedback_msg.feedback)

#         send_goal_future = self.action_client_.send_goal_async(
#             goal, feedback_callback=on_feedback
#         )

#         self.get_logger().info('[4] 等待控制器接受...')
#         while not send_goal_future.done():
#             time.sleep(0.01)

#         gh_result = send_goal_future.result()
#         if not gh_result.accepted:
#             self.get_logger().warn('[5] 控制器拒绝了轨迹')
#             goal_handle.abort()
#             result = FollowJointTrajectory.Result()
#             result.error_code = -1
#             return result

#         self.get_logger().info('[5] 控制器已接受，等待执行完成...')
#         result_future = gh_result.get_result_async()

#         while not result_future.done():
#             time.sleep(0.01)

#         result = result_future.result().result
#         self.get_logger().info(f'[6] 执行完成，error_code={result.error_code}')

#         if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
#             goal_handle.succeed()
#         else:
#             goal_handle.abort()

#         return result


# def main(args=None):
#     rclpy.init(args=args)
#     node = TrajectoryProxy()
#     executor = MultiThreadedExecutor()
#     rclpy.spin(node, executor=executor)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


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
            self.get_logger().warn('取消已转发给下游')
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

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return FollowJointTrajectory.Result()

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
        if not goal_handle.is_active: 
            ctx.event.set()
            return

        ctx.result=future.result().result

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