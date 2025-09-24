import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import time
import math

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
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

        # MAVROS Publisher
        self.setpoint_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.setpoint_timer = self.create_timer(1.0 / 20.0, self._publish_setpoint, callback_group=self.callback_group)
        self.setpoint_timer.cancel()
        self._target_pose = PoseStamped()

        # Subscriber to our internal state topic
        self.create_subscription(DroneState, '/indoor_drone/state', self._state_callback, 10, callback_group=self.callback_group)

        # Action Servers
        self.arm_action_server = ActionServer(self, Arm, 'drone/arm', self._arm_execute_callback, callback_group=self.callback_group)
        self.takeoff_action_server = ActionServer(self, Takeoff, 'drone/takeoff', self._takeoff_execute_callback, callback_group=self.callback_group)
        self.land_action_server = ActionServer(self, Land, 'drone/land', self._land_execute_callback, callback_group=self.callback_group)
        self.goto_position_action_server = ActionServer(self, GotoPosition, 'drone/goto_position', self._goto_position_execute_callback, callback_group=self.callback_group)

        self.get_logger().info("Action servers created and ready.")

    def _state_callback(self, msg): self._current_state = msg
    def _publish_setpoint(self):
        self._target_pose.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_publisher.publish(self._target_pose)

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
        self._target_pose = goal_handle.request.target_pose
        self.setpoint_timer.reset()

        feedback_msg = GotoPosition.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.setpoint_timer.cancel()
                return GotoPosition.Result(success=False)

            if not self._current_state:
                await asyncio.sleep(0.1)
                continue

            pos = self._current_state.pose.position
            target_pos = self._target_pose.pose.position
            distance = math.sqrt((pos.x - target_pos.x)**2 + (pos.y - target_pos.y)**2 + (pos.z - target_pos.z)**2)

            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)

            if distance < 0.2: # Goal reached
                break

            await asyncio.sleep(0.1)

        self.setpoint_timer.cancel()
        goal_handle.succeed()
        return GotoPosition.Result(success=True)

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
