import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio

from indoor_drone_nav_v2.action import ExecuteMission
from indoor_drone_nav_v2.drone_interfaces.mavros_interface import MAVROSInterface

class MissionExecutorNode(Node):
    """
    This node executes a mission plan by taking a MissionPlan message and
    sending a sequence of commands to the drone via the MAVROSInterface client library.
    """
    def __init__(self):
        super().__init__('mission_executor_node')
        self.get_logger().info("Mission Executor Node starting up...")

        # The drone interface is a client library that uses this node to communicate
        self.drone_interface = MAVROSInterface(self)

        self.action_server = ActionServer(
            self,
            ExecuteMission,
            'drone/execute_mission',
            execute_callback=self._execute_mission_callback,
            callback_group=ReentrantCallbackGroup() # Use reentrant group for async calls
        )

    async def _execute_mission_callback(self, goal_handle):
        self.get_logger().info("Executing mission...")

        feedback_msg = ExecuteMission.Feedback()
        mission_plan = goal_handle.request.mission_plan

        # Ensure we are connected to the drone before starting
        if not self.drone_interface.state.connected:
            if not await self.drone_interface.connect():
                self.get_logger().error("Could not connect to drone. Aborting mission.")
                goal_handle.abort()
                return ExecuteMission.Result(success=False)

        for i, waypoint in enumerate(mission_plan.waypoints):
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Mission canceled by client.")
                goal_handle.canceled()
                return ExecuteMission.Result(success=False)

            # --- Publish Feedback ---
            self.get_logger().info(f"Moving to waypoint {i+1}/{len(mission_plan.waypoints)}...")
            feedback_msg.current_waypoint_index = i
            feedback_msg.status = f"Moving to waypoint {i}"
            goal_handle.publish_feedback(feedback_msg)

            # --- Execute Waypoint ---
            pos = waypoint.pose.position
            # The goto_position method now handles the action client call
            success = await self.drone_interface.goto_position(pos.x, pos.y, pos.z)

            if not success:
                self.get_logger().error(f"Failed to execute goto_position for waypoint {i}.")
                goal_handle.abort()
                return ExecuteMission.Result(success=False)

            # In a real implementation, we would wait for the goto_position action to complete.
            # The current client library's goto_position is simplified and returns immediately.
            # We will add a simple delay to simulate travel time.
            self.get_logger().info(f"Waiting for waypoint {i+1} to be reached (simulated)...")
            await asyncio.sleep(5.0)

        self.get_logger().info("Mission execution successful.")
        goal_handle.succeed()
        return ExecuteMission.Result(success=True)

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
