import asyncio

# This is a conceptual example of how the system could be used programmatically.
# It assumes that the necessary ROS2 nodes are running.

# In a real ROS2 script, you would use rclpy to create nodes, publishers, subscribers, etc.
# For simplicity, this example just shows the high-level logic.

from indoor_drone_nav_v2.src.indoor_drone_nav_v2.drone_interfaces.universal_interface import MAVROSInterface
from indoor_drone_nav_v2.src.indoor_drone_nav_v2.mission_planning.intelligent_mission_planner import IntelligentMissionPlanner

async def main():
    print("--- Basic Navigation Example ---")

    # 1. Initialize the interface to the drone
    # In a real script, this would likely be a client to a running ROS2 node.
    drone = MAVROSInterface(config={})

    # 2. Connect to the drone
    await drone.connect()
    if not drone.get_state().connected:
        print("Failed to connect to drone.")
        return

    # 3. Arm and Takeoff
    await drone.arm()
    await drone.takeoff(altitude=1.5)
    print("Takeoff complete.")
    await asyncio.sleep(2)

    # 4. Go to a specific position
    print("Navigating to position (5, 2, 1.5)...")
    await drone.goto_position(x=5.0, y=2.0, z=1.5)
    print("Position reached.")
    await asyncio.sleep(2)

    # 5. Use the mission planner to create a simple mission
    planner = IntelligentMissionPlanner(config={})
    mission_request = {
        'type': 'custom',
        'waypoints': [
            {'position': (5, -2, 1.5)},
            {'position': (0, -2, 1.5)},
            {'position': (0, 0, 1.5)},
        ]
    }
    mission_plan = await planner.plan_mission(mission_request)
    print(f"Mission planned with {len(mission_plan.waypoints)} waypoints.")

    # 6. Execute the mission
    for i, waypoint in enumerate(mission_plan.waypoints):
        print(f"Executing waypoint {i+1}: {waypoint.position}")
        await drone.goto_position(*waypoint.position)
        await asyncio.sleep(1)

    print("Mission complete.")

    # 7. Land
    print("Landing...")
    await drone.land()
    print("Landed safely.")

if __name__ == "__main__":
    # Note: This script is for demonstration purposes.
    # To run it, you would need a running simulation or a real drone
    # with all the ROS2 nodes from this package active.
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted by user.")
