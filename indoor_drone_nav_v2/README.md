# Indoor Drone Navigation System V2 - Production-Ready Architecture

## 🎯 Project Overview
A comprehensive, production-ready indoor autonomous navigation system that works with any drone platform, featuring advanced SLAM, intelligent mission planning, robust safety systems, and professional-grade monitoring tools.

## 🚀 How to Get Started with V2

### 1. Clone the repository
```bash
git clone https://github.com/your-repo/indoor_drone_nav_v2.git
cd indoor_drone_nav_v2
```

### 2. Install Dependencies
Install Python dependencies:
```bash
pip install -r requirements.txt
```
This project is based on ROS2 Humble. Please ensure you have a working ROS2 installation and have sourced your environment (e.g., `source /opt/ros/humble/setup.bash`).

### 3. One-command Setup
Use the quick start script to configure the system for your drone. For example, to set up for a basic PX4 drone:
```bash
python3 scripts/quick_start.py --setup px4_basic --test --gui
```
This will copy the correct preset configuration files into the active `config/` subdirectories.

### 4. Build and Source the Workspace
Use `colcon` to build the ROS2 workspace:
```bash
colcon build
```
After the build is complete, source the new setup file:
```bash
source install/setup.bash
```

### 5. Launch the System
Launch the system using the appropriate ROS2 launch file:
```bash
ros2 launch indoor_drone_nav_v2 px4_basic.launch.py
```

### 6. Open Web Interface
If you launched with `gui:=true`, you can run the web server. In a separate terminal (with the workspace sourced), run:
```bash
uvicorn src.indoor_drone_nav_v2.gui_interface.mission_control_server:app --host 0.0.0.0 --port 8080
```
Then, open your browser to [http://localhost:8080](http://localhost:8080).


## 🎯 V2 Key Improvements Summary

What Makes V2 Production-Ready:

- **🔧 Universal Compatibility** - Works with ANY drone through abstraction layers
- **🛡️ Advanced Safety** - Multi-layered safety with predictive capabilities
- **🧠 Intelligent Systems** - AI-powered mission planning and adaptive navigation
- **💻 Professional GUI** - Web-based control interface with real-time monitoring
- **📊 Complete Analytics** - Comprehensive logging, performance analysis, and maintenance prediction
- **🧪 Thorough Testing** - Automated test suites covering all scenarios
- **🚀 Easy Deployment** - One-command setup for any platform
- **📱 Production Features** - Error recovery, data management, scalable architecture

## 🏗️ Final Project Structure
```
indoor_drone_nav_v2/
├── 📄 README.md
├── 📄 requirements.txt
├── 📄 package.xml
├── 📄 CMakeLists.txt
├── 📄 setup.py
│
├── 📁 src/indoor_drone_nav_v2/
│   ├── 📁 core/
│   ├── 📁 drone_interfaces/
│   └── ... (and other modules)
│
├── 📁 config/
│   ├── 📁 presets/
│   └── ... (and other configs)
│
├── 📁 launch/
│   ├── 📄 master_launch.py
│   └── ... (and other launch files)
│
├── 📁 scripts/
│   ├── 📄 quick_start.py
│   └── ... (and other scripts)
│
├── 📁 testing_framework/
├── 📁 Docker/
├── 📁 docs/
└── 📁 examples/
```
