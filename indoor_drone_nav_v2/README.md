# Indoor Drone Navigation System V2

## 🎯 Project Overview
A comprehensive, production-ready indoor autonomous navigation system that works with any drone platform. This project is built on ROS2 and features a scalable microservice architecture, advanced SLAM capabilities provided by `slam_toolbox`, intelligent mission planning, robust safety systems, and a professional web-based monitoring and control interface.

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

## 🔧 System Modules In-Depth

This section provides details on the intended functionality of each major module in the `src/indoor_drone_nav_v2` directory.

*   **`/core`**: Contains the fundamental, non-ROS-specific algorithms.
    *   `path_planner`: Placeholder for global and local path planning algorithms.

*   **`/drone_interfaces`**: The ROS2 layer for drone communication.
    *   `state_aggregator_node.py`: Subscribes to various sensor topics and uses the TF2 tree to determine the drone's pose in the `map` frame. It publishes the aggregated `DroneState`.
    *   `drone_action_server_node.py`: Provides high-level ROS2 Actions for drone control.
    *   `mavros_interface.py`: A Python client library for easily interacting with the action servers.

*   **`/mission_planning`**: Handles high-level mission logic.
    *   `intelligent_mission_planner.py`: Contains classes for generating waypoints and optimizing mission paths.
    *   `mission_executor_node.py`: A ROS2 node that executes mission plans.

*   **`/safety_systems`**:
    *   `advanced_safety_monitor.py`: A placeholder for a system that would integrate various safety checks like collision prediction and human detection.

*   **`/gui_interface`**:
    *   `mission_control_server.py`: A ROS2 node that runs the FastAPI web server, bridging the web UI with the ROS2 system.

---

## ⚙️ Configuration System

The system uses YAML files located in the `/config` directory.

*   **`/config/presets`**: Contains complete configuration files for different setups (e.g., `px4_quadcopter_basic.yaml`, `standard_safety.yaml`).
*   **`/config/[drones|sensors|etc.]`**: These directories hold the *active* configuration files.
*   **`scripts/quick_start.py`**: This script works by copying the desired presets into the active configuration directories. For example, running with `--setup px4_basic` will copy the `px4_quadcopter_basic.yaml` preset to `config/drones/active.yaml`.

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

## 🐳 Deployment with Docker

This project includes a Docker-based deployment system for reproducibility.

*   **`Dockerfile.jetson_nano`**: Builds a container with optimizations for NVIDIA Jetson Nano.
*   **`Dockerfile.desktop`**: A placeholder for a standard desktop build.
*   **`docker-compose.yml`**: Orchestrates the services. To run the system with Docker (mock hardware access):
    ```bash
    # This will build and run the 'indoor_drone_nav' service
    docker-compose up indoor_drone_nav
    ```

---

## 💻 For Developers

*   **Adding a new action:** Define a new `.action` file in the `/action` directory, add it to `CMakeLists.txt`, rebuild the workspace with `colcon build`, and then implement the server logic in `drone_action_server_node.py`.
*   **Running Tests:** The testing framework includes unit and integration tests.
    *   **Unit Tests:** Located in `/testing_framework/unit_tests`. Run with `pytest`:
        ```bash
        pytest testing_framework/unit_tests/
        ```
    *   **Integration Tests:** Located in `/testing_framework/integration_tests`. These tests launch the full system, so they are run using `colcon`:
        ```bash
        colcon test --packages-select indoor_drone_nav_v2
        ```
