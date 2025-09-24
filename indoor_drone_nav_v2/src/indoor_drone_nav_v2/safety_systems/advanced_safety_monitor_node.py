import rclpy
from rclpy.node import Node
import asyncio

# This will be resolved by the python path setup in a ROS2 environment
from indoor_drone_nav_v2.msg import DroneState
from .advanced_safety_monitor import AdvancedSafetyMonitor, SafetyAlert, SafetyLevel, UniversalDroneInterface

class AdvancedSafetyMonitorNode(Node):
    """
    ROS2 Node for the AdvancedSafetyMonitor.
    """

    def __init__(self):
        super().__init__('advanced_safety_monitor_node')
        self.get_logger().info('Advanced Safety Monitor Node started')

        # Instantiate the safety monitor
        # We pass None for the drone_interface as this node's purpose is to run the assessment,
        # not to execute emergency actions in this version.
        self.safety_monitor = AdvancedSafetyMonitor(config={}, drone_interface=None)

        # Subscribe to the drone state topic
        self.subscription = self.create_subscription(
            DroneState,
            '/indoor_drone/state',
            self.drone_state_callback,
            10)
        self.latest_drone_state = None

        # Create a timer to run the safety assessment periodically
        self.timer = self.create_timer(2.0, self.run_safety_check)
        self.loop = asyncio.get_event_loop()

    def drone_state_callback(self, msg):
        """Callback for the drone state subscriber."""
        self.latest_drone_state = msg

    def run_safety_check(self):
        """Periodically run the safety assessment."""
        if self.latest_drone_state is None:
            self.get_logger().info('No drone state received yet. Skipping safety check.')
            return

        # Create mock sensor data and trajectory for now
        # In a real system, these would come from other nodes.
        mock_sensor_data = {'camera': {'color': 'mock_image_data'}}
        mock_planned_trajectory = [(1.0, 1.0, 1.0), (2.0, 2.0, 2.0)]

        # Run the safety assessment
        # Since assess_safety is an async function, we need to run it in an event loop
        alerts = self.loop.run_until_complete(
            self.safety_monitor.assess_safety(
                self.latest_drone_state,
                mock_sensor_data,
                mock_planned_trajectory
            )
        )

        # Log the alerts
        if alerts:
            for alert in alerts:
                self.get_logger().warn(f"Safety Alert: {alert.message} (Level: {alert.level.value})")
        else:
            self.get_logger().info('Safety check complete. No alerts.')


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedSafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
