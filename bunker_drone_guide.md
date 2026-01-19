# GPS-Denied Navigation Software Stack
## Universal Drone Navigation System (Hardware-Agnostic)

---

## Project Overview

**Mission**: Develop a modular, open-source software system that enables any drone to navigate autonomously in GPS-denied environments using LiDAR SLAM and optical flow.

**Key Features**:
- Hardware-agnostic design (works with any drone + sensors)
- Plug-and-play sensor integration
- Real-time SLAM and navigation
- Easy deployment and configuration

---

## Software Architecture

### System Design Philosophy

```
┌─────────────────────────────────────────────────────────────┐
│                    Ground Control Station                    │
│                   (Mission Planning & Monitoring)            │
└───────────────────────────┬─────────────────────────────────┘
                            │ MAVLink/WiFi
┌───────────────────────────┴─────────────────────────────────┐
│                      DRONE HARDWARE                          │
│  ┌────────────────────────────────────────────────────┐     │
│  │         Flight Controller (PX4/ArduPilot)          │     │
│  │              (Low-level stabilization)             │     │
│  └──────────────────┬─────────────────────────────────┘     │
│                     │ MAVROS/px4_ros_com                     │
│  ┌──────────────────┴─────────────────────────────────┐     │
│  │      Companion Computer (Your Software Runs Here)  │     │
│  │                                                     │     │
│  │  ┌───────────────────────────────────────────┐    │     │
│  │  │         ROS 2 Navigation Stack            │    │     │
│  │  │  ┌─────────────────────────────────┐     │    │     │
│  │  │  │   Your Custom Software Layer    │     │    │     │
│  │  │  │  - SLAM Coordinator             │     │    │     │
│  │  │  │  - Sensor Fusion Manager        │     │    │     │
│  │  │  │  - Mission Controller           │     │    │     │
│  │  │  │  - Safety Monitor                │     │    │     │
│  │  │  └─────────────────────────────────┘     │    │     │
│  │  │                                           │    │     │
│  │  │  ┌──────────┐  ┌──────────┐  ┌────────┐ │    │     │
│  │  │  │  SLAM    │  │  Sensor  │  │  Path  │ │    │     │
│  │  │  │  Engine  │  │  Fusion  │  │ Planner│ │    │     │
│  │  │  └──────────┘  └──────────┘  └────────┘ │    │     │
│  │  └───────────────────────────────────────────┘    │     │
│  └─────────────────────────────────────────────────────┘     │
│                     │                                         │
│  ┌──────────────────┴─────────────────────────────────┐     │
│  │              Sensor Inputs                         │     │
│  │  LiDAR │ Optical Flow │ IMU │ Cameras │ Others    │     │
│  └────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
```

---

## Development Environment Setup

### Prerequisites

#### Operating System
- **Ubuntu 22.04 LTS** (recommended) or Ubuntu 20.04 LTS
- Minimum 4GB RAM, 50GB disk space
- Can be installed on: Raspberry Pi 4, Jetson Nano/Orin, x86 laptop

#### Install ROS 2 Humble (Ubuntu 22.04)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Install Essential Development Tools

```bash
# Build tools
sudo apt install python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Git and version control
sudo apt install git git-lfs

# Python dependencies
sudo apt install python3-pip
pip3 install --upgrade setuptools
```

---

## Software Stack Components

### 1. Flight Controller Interface

#### Option A: MAVROS (MAVLink-ROS Bridge)

```bash
# Install MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

**Configuration File**: `mavros_config.yaml`
```yaml
# MAVROS configuration for GPS-denied mode
mavros:
  fcu_url: "/dev/ttyACM0:57600"  # Serial connection to flight controller
  gcs_url: ""
  target_system_id: 1
  target_component_id: 1
  
  # Enable necessary plugins
  plugin_allowlist:
    - sys_status
    - sys_time
    - imu
    - local_position
    - setpoint_position
    - setpoint_velocity
    - command
    - rc_io
    - vision_pose
    - vision_speed
    - distance_sensor
    - optical_flow
```

#### Option B: px4_ros_com (XRCE-DDS for PX4)

```bash
# Clone PX4-ROS2 bridge
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git

# Build
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 2. SLAM Implementation Options

#### Option A: Cartographer (2D/3D SLAM - Google)

**Best for**: RPLidar, stable and mature

```bash
# Install Cartographer
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# Clone configuration packages
cd ~/ros2_ws/src
git clone https://github.com/ros2/cartographer_ros.git -b humble
```

**Cartographer Configuration**: `cartographer_config.lua`
```lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,  -- No GPS
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
```

#### Option B: FAST-LIO2 (3D LiDAR-Inertial)

**Best for**: Livox LiDAR, real-time performance

```bash
cd ~/ros2_ws/src
git clone https://github.com/Ericsii/FAST_LIO.git -b humble

# Install dependencies
sudo apt install libeigen3-dev libpcl-dev

cd ~/ros2_ws
colcon build --packages-select fast_lio
```

#### Option C: Hector SLAM (Lightweight 2D)

**Best for**: Low-compute devices like Raspberry Pi

```bash
cd ~/ros2_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git -b humble-devel

cd ~/ros2_ws
colcon build --packages-select hector_slam
```

#### Option D: RTAB-Map (Visual + LiDAR)

**Best for**: Multi-sensor fusion with cameras

```bash
sudo apt install ros-humble-rtabmap-ros
```

---

### 3. Sensor Fusion (robot_localization)

```bash
# Install robot_localization
sudo apt install ros-humble-robot-localization
```

**EKF Configuration**: `ekf_config.yaml`
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    publish_acceleration: false
    
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # Optical Flow Odometry
    odom0: /optical_flow/odom
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  true,  false,   # vx, vy
                   false, false, true,    # vyaw
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    
    # IMU Data
    imu0: /mavros/imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,     # orientation
                  false, false, false,
                  true,  true,  true,     # angular velocity
                  true,  true,  true]     # linear acceleration
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
    
    # SLAM Pose Estimate
    pose0: /slam/pose
    pose0_config: [true,  true,  true,    # x, y, z
                   false, false, true,    # yaw only
                   false, false, false,
                   false, false, false,
                   false, false, false]
    pose0_differential: false
    pose0_relative: false
    
    # Process noise covariance
    process_noise_covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.015]
```

---

### 4. Navigation Stack (Nav2)

```bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

**Nav2 Configuration**: `nav2_params.yaml`
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: false
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
      
    general_goal_checker:
      stateful: True
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 2.5
      acc_lim_y: 0.0
      acc_lim_theta: 3.2
      decel_lim_x: -2.5
      decel_lim_y: 0.0
      decel_lim_theta: -3.2
      
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

---

### 5. Sensor Drivers

#### LiDAR Drivers

**RPLidar**:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws
colcon build --packages-select sllidar_ros2
```

**Livox**:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select livox_ros2_driver
```

#### Optical Flow / Camera Drivers

**Generic USB Camera**:
```bash
sudo apt install ros-humble-usb-cam
```

**Intel RealSense**:
```bash
sudo apt install ros-humble-realsense2-camera
```

---

## Project Structure

### Create Your Workspace

```bash
# Create workspace
mkdir -p ~/bunker_nav_ws/src
cd ~/bunker_nav_ws/src

# Your custom package
ros2 pkg create --build-type ament_python bunker_navigation
cd bunker_navigation

# Create directory structure
mkdir -p launch config scripts maps logs
```

### Directory Layout
```
bunker_nav_ws/
├── src/
│   └── bunker_navigation/
│       ├── bunker_navigation/
│       │   ├── __init__.py
│       │   ├── slam_manager.py          # SLAM coordinator
│       │   ├── mission_controller.py    # Mission logic
│       │   ├── safety_monitor.py        # Safety checks
│       │   └── sensor_bridge.py         # Sensor interface
│       ├── launch/
│       │   ├── bunker_full.launch.py    # Master launch file
│       │   ├── slam_only.launch.py      # SLAM testing
│       │   └── navigation.launch.py     # Nav2 launch
│       ├── config/
│       │   ├── ekf_config.yaml
│       │   ├── nav2_params.yaml
│       │   ├── cartographer_config.lua
│       │   └── sensors.yaml
│       ├── scripts/
│       │   ├── test_sensors.py
│       │   ├── calibrate.py
│       │   └── visualize.py
│       ├── maps/                        # Saved maps
│       ├── logs/                        # Flight logs
│       ├── package.xml
│       └── setup.py
└── install/
```

---

## Core Software Modules

### Module 1: SLAM Manager (`slam_manager.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class SLAMManager(Node):
    """
    Manages SLAM system and publishes fused pose estimates
    """
    def __init__(self):
        super().__init__('slam_manager')
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, 
            '/slam/pose', 
            10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # SLAM state
        self.current_pose = None
        self.map_quality = 0.0
        self.is_localized = False
        
        # Timers
        self.create_timer(0.1, self.publish_pose)  # 10Hz
        
        self.get_logger().info('SLAM Manager initialized')
    
    def scan_callback(self, msg):
        """Process LiDAR scans"""
        # Quality check
        valid_points = sum(1 for r in msg.ranges 
                          if msg.range_min < r < msg.range_max)
        self.map_quality = valid_points / len(msg.ranges)
        
        if self.map_quality < 0.3:
            self.get_logger().warn('Low map quality - featureless environment')
    
    def publish_pose(self):
        """Publish current pose estimate"""
        if self.current_pose is not None:
            self.pose_pub.publish(self.current_pose)

def main(args=None):
    rclpy.init(args=args)
    slam_manager = SLAMManager()
    rclpy.spin(slam_manager)
    slam_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Module 2: Mission Controller (`mission_controller.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np

class MissionController(Node):
    """
    High-level mission control and waypoint navigation
    """
    def __init__(self):
        super().__init__('mission_controller')
        
        # State tracking
        self.current_state = None
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_idx = 0
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/odometry/filtered',
            self.pose_callback,
            10
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Services
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
        
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )
        
        # Mission timer
        self.create_timer(0.05, self.mission_loop)  # 20Hz
        
        self.get_logger().info('Mission Controller initialized')
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def pose_callback(self, msg):
        self.current_pose = msg
    
    def load_waypoints(self, waypoint_list):
        """Load waypoints for autonomous mission"""
        self.waypoints = waypoint_list
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Loaded {len(waypoint_list)} waypoints')
    
    def mission_loop(self):
        """Main mission execution loop"""
        if self.current_state is None or self.current_pose is None:
            return
        
        # Check if we have waypoints
        if not self.waypoints:
            return
        
        # Navigate to current waypoint
        target = self.waypoints[self.current_waypoint_idx]
        self.setpoint_pub.publish(target)
        
        # Check if reached waypoint
        if self.reached_waypoint(target):
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info('Mission complete!')
                self.waypoints = []
    
    def reached_waypoint(self, target, tolerance=0.3):
        """Check if current position is within tolerance of target"""
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        dz = self.current_pose.pose.position.z - target.pose.position.z
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        return distance < tolerance

def main(args=None):
    rclpy.init(args=args)
    mission_controller = MissionController()
    rclpy.spin(mission_controller)
    mission_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Module 3: Safety Monitor (`safety_monitor.py`)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np

class SafetyMonitor(Node):
    """
    Monitors drone safety and triggers emergency actions
    """
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Safety thresholds
        self.declare_parameter('battery_critical', 20.0)
        self.declare_parameter('battery_warning', 30.0)
        self.declare_parameter('max_position_drift', 2.0)
        self.declare_parameter('min_obstacle_distance', 0.5)
        
        # State
        self.battery_percent = 100.0
        self.last_pose_time = self.get_clock().now()
        self.localization_lost = False
        self.obstacle_too_close = False
        
        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/odometry/filtered',
            self.pose_callback,
            10
        )
        
        # Service clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Safety check timer
        self.create_timer(1.0, self.safety_check)  # 1Hz
        
        self.get_logger().info('Safety Monitor active')
    
    def battery_callback(self, msg):
        self.battery_percent = msg.percentage * 100.0
    
    def scan_callback(self, msg):
        """Check for obstacles"""
        min_distance = min([r for r in msg.ranges 
                           if msg.range_min < r < msg.range_max], 
                           default=float('inf'))
        
        threshold = self.get_parameter('min_obstacle_distance').value
        self.obstacle_too_close = min_distance < threshold
    
    def pose_callback(self, msg):
        self.last_pose_time = self.get_clock().now()
        self.localization_lost = False
    
    def safety_check(self):
        """Periodic safety checks"""
        # Check battery
        battery_critical = self.get_parameter('battery_critical').value
        battery_warning = self.get_parameter('battery_warning').value
        
        if self.battery_percent < battery_critical:
            self.get_logger().error('CRITICAL BATTERY - Emergency landing!')
            self.trigger_emergency_land()
        elif self.battery_percent < battery_warning:
            self.get_logger().warn(f'Low battery: {self.battery_percent:.1f}%')
        
        # Check localization
        time_since_pose = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if time_since_pose > 2.0:
            self.localization_lost = True
            self.get_logger().error('Localization lost - Emergency landing!')
            self.trigger_emergency_land()
        
        # Check obstacles
        if self.obstacle_too_close:
            self.get_logger().warn('Obstacle detected - Stopping')
    
    def trigger_emergency_land(self):
        """Switch to emergency landing mode"""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.set_mode_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Master Launch File

**File**: `launch/bunker_full.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('bunker_navigation')
    
    return LaunchDescription([
        # MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[os.path.join(pkg_dir, 'config', 'mavros_config.yaml')]
        ),
        
        # LiDAR Driver (RPLidar example)
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_frequency': 10.0
            }]
        ),
        
        # Cartographer SLAM
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[
                '-configuration_directory', os.path.join(pkg_dir, 'config'),
                '-configuration_basename', 'cartographer_config.lua'
            ]
        ),
        
        # Robot Localization (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg_dir, 'config', 'ekf_config.yaml')]
        ),
        
        # Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('nav2_bringup'), 
                            'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
            }.items()
        ),
        
        # Custom nodes
        Node(
            package='bunker_navigation',
            executable='slam_manager',
            name='slam_manager'
        ),
        
        Node(
            package='bunker_navigation',
            executable='mission_controller',
            name='mission_controller'
        ),
        
        Node(
            package='bunker_navigation',
            executable='safety_monitor',
            name='safety_monitor'
        ),
    ])
```

---

## Testing & Development Workflow

### Phase 1: Simulation Testing (Week 1-2)

#### Setup Gazebo Simulation

```bash
# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Install PX4 SITL
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

#### Test in Simulation

```bash
# Terminal 1: Launch PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic

# Terminal 2: Launch your software stack
ros2 launch bunker_navigation bunker_full.launch.py

# Terminal 3: Send test waypoints
ros2 topic pub /waypoint geometry_msgs/PoseStamped "..."
```

### Phase 2: Bench Testing (Week 3)

**Test sensors without flight:**

```bash
# Test LiDAR only
ros2 launch bunker_navigation slam_only.launch.py

# Visualize in RViz
ros2 run rviz2 rviz2

# Record data
ros2 bag record -a -o test_bench
```

### Phase 3: Hardware Integration (Week 4-5)

**Connect to real drone:**

1. Install software on companion computer (Raspberry Pi/Jetson)
2. Connect flight controller via USB/UART
3. Test sensor connections
4. Run launch file
5. Monitor in QGroundControl

```bash
# On drone companion computer
ros2 launch bunker_navigation bunker_full.launch.py

# Monitor from laptop
export ROS_DOMAIN_ID=42
ros2 topic list
ros2 topic echo /mavros/state
```

### Phase 4: Flight Testing (Week 6+)

**Progressive testing:**
1. Indoor hover test (tethered)
2. Position hold test (no GPS)
3. Simple waypoint test (2 points)
4. Corridor navigation
5. Complex environment

---

## Package Configuration Files

### `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>bunker_navigation</name>
  <version>1.0.0</version>
  <description>GPS-denied navigation system for drones</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>mavros_msgs</depend>
  <depend>robot_localization</depend>
  <depend>nav2_bringup</depend>
  <depend>cartographer_ros</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### `setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'bunker_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='GPS-denied navigation for drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_manager = bunker_navigation.slam_manager:main',
            'mission_controller = bunker_navigation.mission_controller:main',
            'safety_monitor = bunker_navigation.safety_monitor:main',
        ],
    },
)
```

---

## Building & Running

### Build Your Package

```bash
cd ~/bunker_nav_ws
colcon build --packages-select bunker_navigation
source install/setup.bash
```

### Run the System

```bash
# Full system
ros2 launch bunker_navigation bunker_full.launch.py

# SLAM only (for testing)
ros2 launch bunker_navigation slam_only.launch.py

# With custom parameters
ros2 launch bunker_navigation bunker_full.launch.py battery_critical:=25.0
```

### Monitor System

```bash
# Check active nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor pose
ros2 topic echo /odometry/filtered

# Monitor battery
ros2 topic echo /mavros/battery

# Visualize
ros2 run rviz2 rviz2
```

---

## Deployment Instructions

### Installing on Companion Computer

```bash
# On Raspberry Pi / Jetson Nano
# 1. Flash Ubuntu 22.04
# 2. Install ROS 2 Humble (see setup section)
# 3. Clone your package

cd ~
mkdir -p bunker_nav_ws/src
cd bunker_nav_ws/src
git clone https://github.com/yourusername/bunker_navigation.git

# 4. Install dependencies
cd ~/bunker_nav_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build
colcon build

# 6. Setup auto-start
sudo nano /etc/systemd/system/bunker-nav.service
```

**Systemd Service File**: `/etc/systemd/system/bunker-nav.service`

```ini
[Unit]
Description=Bunker Navigation System
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/bunker_nav_ws
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/bunker_nav_ws/install/setup.bash && ros2 launch bunker_navigation bunker_full.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
# Enable auto-start
sudo systemctl daemon-reload
sudo systemctl enable bunker-nav.service
sudo systemctl start bunker-nav.service

# Check status
sudo systemctl status bunker-nav.service
```

---

## Performance Optimization

### For Raspberry Pi 4

```bash
# Use lightweight SLAM
# Choose Hector SLAM instead of Cartographer

# Reduce sensor rates
# In sensor config:
scan_frequency: 5.0  # Instead of 10.0

# Limit RViz visualization
# Disable on companion computer, visualize from ground station

# Optimize Python
# Use PyPy3 for better performance (optional)
```

### For Jetson Nano

```bash
# Use power mode 0 (max performance)
sudo nvpmodel -m 0
sudo jetson_clocks

# Use CUDA acceleration (if using visual SLAM)
# Install CUDA-enabled OpenCV

# Monitor temperature
sudo tegrastats
```

---

## Troubleshooting Guide

### Common Issues

**Issue**: No pose output from SLAM
- Check LiDAR data: `ros2 topic echo /scan`
- Verify scan has valid ranges
- Check SLAM node logs: `ros2 node info /cartographer_node`

**Issue**: EKF not converging
- Verify sensor timestamps are synchronized
- Check covariance matrices in config
- Ensure all sensors publishing at expected rates

**Issue**: Navigation stuck or not moving
- Check costmap: `ros2 topic echo /local_costmap/costmap`
- Verify goal is reachable
- Check for obstacle inflation radius

**Issue**: High CPU usage
- Reduce sensor update rates
- Simplify SLAM configuration
- Use lightweight algorithms

**Issue**: Lost localization during flight
- Increase IMU trust in sensor fusion
- Add more features to environment
- Reduce maximum velocity

---

## Validation & Metrics

### Key Performance Indicators

**SLAM Quality:**
- Map consistency score > 0.8
- Loop closure success rate > 90%
- Position drift < 1m per 100m traveled

**Navigation Performance:**
- Waypoint accuracy < 0.3m
- Path smoothness (no oscillations)
- Obstacle avoidance success rate > 95%

**System Reliability:**
- Uptime > 99% during mission
- Sensor fusion latency < 50ms
- Safety trigger response < 100ms

### Testing Metrics

```python
# Example metrics collection node
class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        
        self.start_time = self.get_clock().now()
        self.waypoints_reached = 0
        self.total_distance = 0.0
        self.slam_quality_sum = 0.0
        self.samples = 0
        
    def publish_report(self):
        flight_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        avg_slam_quality = self.slam_quality_sum / self.samples if self.samples > 0 else 0
        
        self.get_logger().info(f"""
        === Flight Report ===
        Duration: {flight_time:.1f}s
        Distance: {self.total_distance:.1f}m
        Waypoints: {self.waypoints_reached}
        Avg SLAM Quality: {avg_slam_quality:.2f}
        """)
```

---

## Next Steps & Enhancements

### Immediate (Week 1-4)
1. Complete basic setup and simulation testing
2. Integrate your chosen SLAM algorithm
3. Test sensor fusion in simulation
4. Validate on hardware bench test

### Short-term (Month 2-3)
1. Field test in controlled environment
2. Tune parameters for your hardware
3. Add mission planning interface
4. Implement data logging

### Long-term (Month 4+)
1. Multi-drone coordination
2. Advanced path planning (3D)
3. Machine learning for obstacle detection
4. Swarm navigation capabilities

---

## Resources & Community

### Documentation
- ROS 2 Humble: https://docs.ros.org/en/humble/
- Nav2: https://navigation.ros.org/
- PX4: https://docs.px4.io/
- Cartographer: https://google-cartographer-ros.readthedocs.io/

### GitHub Examples
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ros-planning/navigation2
- https://github.com/cartographer-project/cartographer_ros

### Forums
- ROS Answers: https://answers.ros.org/
- PX4 Discuss: https://discuss.px4.io/
- Robotics Stack Exchange: https://robotics.stackexchange.com/

---

## License

This software uses open-source components under their respective licenses (Apache 2.0, BSD, MIT).

Your custom code can be licensed under MIT or Apache 2.0.

---

**Important Note**: This software is designed to be hardware-agnostic. Simply install it on any companion computer connected to a flight controller with LiDAR and optical flow sensors, configure the sensor topics, and launch!

*Last Updated: January 2026 | Version 1.0*