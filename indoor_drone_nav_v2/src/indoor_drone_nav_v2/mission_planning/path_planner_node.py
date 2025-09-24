import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from indoor_drone_nav_v2.action import GetPath, ExecuteMission
from indoor_drone_nav_v2.msg import MissionPlan, Waypoint
import asyncio

class PathPlannerNode(Node):
    """
    A node that provides a high-level path planning service.
    It receives a goal, uses Nav2 to compute a path, and then sends
    that path to the MissionExecutorNode to be flown.
    """
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().info("Path Planner Node starting...")

        # Action Client for Nav2's ComputePathToPose
        self._nav2_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Action Client for our MissionExecutorNode
        self._mission_client = ActionClient(self, ExecuteMission, '/drone/execute_mission')

        # Action Server for receiving path planning requests
        self._action_server = ActionServer(
            self,
            GetPath,
            '/planner/get_path',
            self.get_path_callback
        )

        self.get_logger().info("Path Planner Node started.")

    async def get_path_callback(self, goal_handle):
        """Callback for the GetPath action server."""
        self.get_logger().info('Received a new path planning request.')
        goal_pose = goal_handle.request.goal_pose

        # 1. Send the goal to Nav2 to get a path
        self.get_logger().info('Requesting path from Nav2...')
        nav2_goal = ComputePathToPose.Goal()
        nav2_goal.goal = goal_pose
        nav2_goal.planner_id = "GridBased"

        path_result = await self._nav2_path_client.send_goal_async(nav2_goal)
        if not path_result.accepted:
            self.get_logger().error('Nav2 rejected the goal.')
            goal_handle.abort()
            return GetPath.Result(success=False)

        path = await path_result.get_result_async()
        if path is None:
            self.get_logger().error('Failed to get path from Nav2.')
            goal_handle.abort()
            return GetPath.Result(success=False)

        self.get_logger().info(f'Received path with {len(path.result.path.poses)} waypoints.')

        # 2. Convert the path to a MissionPlan
        mission_plan = MissionPlan()
        for pose in path.result.path.poses:
            waypoint = Waypoint()
            waypoint.pose = pose.pose
            mission_plan.waypoints.append(waypoint)

        # 3. Send the mission to the MissionExecutor
        self.get_logger().info('Sending mission to executor...')
        mission_goal = ExecuteMission.Goal()
        mission_goal.mission_plan = mission_plan

        mission_result = await self._mission_client.send_goal_async(mission_goal)
        if not mission_result.accepted:
            self.get_logger().error('Mission executor rejected the goal.')
            goal_handle.abort()
            return GetPath.Result(success=False)

        final_result = await mission_result.get_result_async()
        if not final_result.result.success:
            self.get_logger().error('Mission execution failed.')
            goal_handle.abort()
            return GetPath.Result(success=False)

        self.get_logger().info('Path planning and execution successful.')
        goal_handle.succeed()
        return GetPath.Result(success=True)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
