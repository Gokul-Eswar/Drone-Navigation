# Indoor Drone Navigation System V2

## 🎯 Project Overview
A comprehensive, production-ready indoor autonomous navigation system that works with any drone platform. This project is built on ROS2 and features a scalable microservice architecture, advanced SLAM capabilities (placeholder), intelligent mission planning, robust safety systems, and a professional web-based monitoring and control interface.

---

## 🏗️ Architecture Overview

The system is designed as a set of communicating ROS2 nodes (microservices), promoting scalability and modularity.

### Core Microservices:

*   **`StateAggregatorNode`**:
    *   **Responsibility:** Subscribes to raw sensor and state topics from MAVROS (`/mavros/*`).
    *   **Publishes:** A single, clean, aggregated `/indoor_drone/state` topic of type `DroneState.msg`. This provides a single source of truth for the drone's status.

*   **`DroneActionServerNode`**:
    *   **Responsibility:** Exposes high-level control over the drone through ROS2 actions. It translates these abstract goals into low-level MAVROS service calls and commands.
    *   **Actions:** Hosts `/drone/arm`, `/drone/takeoff`, `/drone/land`, and `/drone/goto_position`.

*   **`MissionExecutorNode`**:
    *   **Responsibility:** Executes complex missions composed of multiple waypoints.
    *   **Actions:** Hosts the `/drone/execute_mission` action, which accepts a `MissionPlan.msg`. It uses the `DroneActionServerNode`'s actions to fly the mission.

*   **`GUIServerNode`**:
    *   **Responsibility:** Runs a FastAPI web server and provides a WebSocket bridge to the ROS2 system.
    *   **Function:** Subscribes to `/indoor_drone/state` to display real-time telemetry and uses a client library to call the drone actions based on user interaction in the web UI.

### Key Topics & Actions:

*   **Topic:** `/indoor_drone/state` (`indoor_drone_nav_v2/msg/DroneState`)
    *   Provides the aggregated, real-time state of the drone.
*   **Action:** `/drone/arm` (`indoor_drone_nav_v2/action/Arm`)
    *   Goal-based arming and disarming.
*   **Action:** `/drone/takeoff` (`indoor_drone_nav_v2/action/Takeoff`)
    *   Commands the drone to take off to a specific altitude.
*   **Action:** `/drone/land` (`indoor_drone_nav_v2/action/Land`)
    *   Commands the drone to land at its current position.
*   **Action:** `/drone/goto_position` (`indoor_drone_nav_v2/action/GotoPosition`)
    *   Commands the drone to fly to a specific local position.
*   **Action:** `/drone/execute_mission` (`indoor_drone_nav_v2/action/ExecuteMission`)
    *   Executes a multi-waypoint mission plan.

---

## 🚀 Getting Started

### 1. Prerequisites
*   A working ROS2 Humble installation.
*   `colcon` build tools.
*   Python dependencies (see `requirements.txt`).

### 2. Installation
```bash
# Clone the repository
# git clone https://github.com/your-repo/indoor_drone_nav_v2.git
cd indoor_drone_nav_v2

# Install Python dependencies
pip install -r requirements.txt

# Build the ROS2 workspace. This is crucial for generating the custom messages and actions.
colcon build

# Source the new setup file
source install/setup.bash
```

### 3. Usage

#### Launching the System
The primary launch file brings up all the core microservices for a basic PX4 setup.
```bash
# Make sure you have sourced your workspace (source install/setup.bash)
ros2 launch indoor_drone_nav_v2 px4_basic.launch.py
```

#### Using the Web GUI
The GUI Server node is launched automatically. Open your browser to **[http://localhost:8080](http://localhost:8080)** to monitor and control the drone.

#### Using the Command Line (Example: Takeoff)
The action-based architecture allows you to control the drone directly from the command line, which is great for testing and scripting.

1.  **Arm the drone:**
    ```bash
    ros2 action send_goal /drone/arm indoor_drone_nav_v2/action/Arm "{arm: true}"
    ```
2.  **Take off to 2.5 meters:**
    ```bash
    ros2 action send_goal /drone/takeoff indoor_drone_nav_v2/action/Takeoff "{altitude: 2.5}" --feedback
    ```

---

## 🔧 For Developers

*   **Adding a new action:** Define a new `.action` file in the `/action` directory, add it to `CMakeLists.txt`, rebuild the workspace with `colcon build`, and then implement the server logic in `drone_action_server_node.py`.
*   **Running Tests:** The unit tests are located in `/testing_framework/unit_tests`. You can run them with `pytest`:
    ```bash
    pytest testing_framework/unit_tests/
    ```
    *Note: Tests require a sourced ROS2 environment to find `rclpy`, but use mocking and do not require a live drone connection.*
