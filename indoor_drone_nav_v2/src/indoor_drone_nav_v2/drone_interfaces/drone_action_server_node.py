import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import math

import asyncio
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, Twist
from indoor_drone_nav_v2.action import Arm, Takeoff, Land, GotoPosition
from indoor_drone_nav_v2.msg import DroneState

class DroneActionServerNode(Node):
    """
    Hosts ROS2 Action Servers for high-level drone commands like arm, takeoff, and land.
    """
    def __init__(self):
        super().__init__('drone_action_server_node')
        self.get_logger().info("Drone Action Server Node starting up...")

        self._current_state = None

        # Use a reentrant callback group to allow service clients and action callbacks
        # to be called from within other callbacks without deadlocking.
        self.callback_group = ReentrantCallbackGroup()

        # MAVROS Service Clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming', callback_group=self.callback_group)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode', callback_group=self.callback_group)
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff', callback_group=self.callback_group)
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land', callback_group=self.callback_group)

        # Velocity Publisher for the Goto action
        self.goto_cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel/goto', 10)

        # Subscriber to our internal state topic
        self.create_subscription(DroneState, '/indoor_drone/state', self._state_callback, 10, callback_group=self.callback_group)

        # Action Servers
        self.arm_action_server = ActionServer(self, Arm, 'drone/arm', self._arm_execute_callback, callback_group=self.callback_group)
        self.takeoff_action_server = ActionServer(self, Takeoff, 'drone/takeoff', self._takeoff_execute_callback, callback_group=self.callback_group)
        self.land_action_server = ActionServer(self, Land, 'drone/land', self._land_execute_callback, callback_group=self.callback_group)
        self.goto_position_action_server = ActionServer(self, GotoPosition, 'drone/goto_position', self._goto_position_execute_callback, callback_group=self.callback_group)

        self.get_logger().info("Action servers created and ready.")

    def _state_callback(self, msg): self._current_state = msg

    async def _arm_execute_callback(self, goal_handle):
        self.get_logger().info(f"Arm action received: arm={goal_handle.request.arm}")
        req = CommandBool.Request(value=goal_handle.request.arm)
        future = self.arming_client.call_async(req)
        await future

        result = Arm.Result()
        if future.result() and future.result().success:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
        return result

    async def _takeoff_execute_callback(self, goal_handle):
        self.get_logger().info(f"Takeoff action received: altitude={goal_handle.request.altitude}")
        if not self._current_state or not self._current_state.armed:
            self.get_logger().error("Takeoff aborted: drone is not armed.")
            goal_handle.abort()
            return Takeoff.Result(success=False)

        req = CommandTOL.Request(altitude=goal_handle.request.altitude)
        future = self.takeoff_client.call_async(req)
        await future

        result = Takeoff.Result()
        if future.result() and future.result().success:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
        return result

    async def _land_execute_callback(self, goal_handle):
        self.get_logger().info("Land action received.")
        req = CommandTOL.Request()
        future = self.land_client.call_async(req)
        await future

        result = Land.Result()
        if future.result() and future.result().success:
            goal_handle.succeed()
            result.success = True
        else:
            goal_handle.abort()
        return result

    async def _goto_position_execute_callback(self, goal_handle):
        self.get_logger().info("GotoPosition action received.")
        target_pose = goal_handle.request.target_pose

        # P-controller gains
        k_p = 0.5
        max_vel = 0.5

        feedback_msg = GotoPosition.Feedback()

        # Using a timer for the control loop is better than a while loop
        # as it doesn't block the executor.
        self.control_timer = self.create_timer(1.0 / 20.0, self._goto_control_loop, callback_group=self.callback_group)
        self.control_timer_goal_handle = goal_handle
        self.control_timer_target_pose = target_pose

    def _goto_control_loop(self):
        goal_handle = self.control_timer_goal_handle
        target_pose = self.control_timer_target_pose

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.goto_cmd_vel_publisher.publish(Twist()) # Stop the drone
            self.control_timer.cancel()
            return

        if not self._current_state:
            return

        current_pos = self._current_state.pose.position
        target_pos = target_pose.pose.position

        error_x = target_pos.x - current_pos.x
        error_y = target_pos.y - current_pos.y
        error_z = target_pos.z - current_pos.z

        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)

        feedback_msg = GotoPosition.Feedback()
        feedback_msg.distance_to_goal = distance
        goal_handle.publish_feedback(feedback_msg)

        if distance < 0.2: # Goal reached
            self.goto_cmd_vel_publisher.publish(Twist()) # Stop the drone
            goal_handle.succeed()
            result = GotoPosition.Result(success=True)
            self.control_timer.cancel()
            # This is not ideal, but we need to set the result for the action
            # A better approach would be to use a future.
            goal_handle._set_result(result)
            return

        twist_msg = Twist()
        twist_msg.linear.x = min(0.5, 0.5 * error_x)
        twist_msg.linear.y = min(0.5, 0.5 * error_y)
        twist_msg.linear.z = min(0.5, 0.5 * error_z)
        self.goto_cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneActionServerNode()
    # Use a MultiThreadedExecutor to handle callbacks concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
