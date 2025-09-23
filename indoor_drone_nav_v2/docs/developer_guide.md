# Indoor Drone Navigation System V2 - Developer Guide

This guide provides information for developers working on the Indoor Drone Navigation System.

## 1. Code Structure
The source code is located in the `src/indoor_drone_nav_v2` directory.

- `core/`: Core algorithms for SLAM, path planning, etc.
- `drone_interfaces/`: Abstraction layer for communicating with different drones.
- `sensor_processing/`: Nodes for processing data from various sensors.
- `safety_systems/`: Safety monitoring and emergency response systems.
- `mission_planning/`: Intelligent mission planning and optimization.
- `gui_interface/`: The FastAPI web server for the GUI.
- `data_management/`: Data logging, storage, and analytics.
- `ml_modules/`: Machine learning modules for advanced capabilities.

## 2. Building the Code
This is a ROS2 package. Use `colcon` to build the code:
```bash
colcon build
```

## 3. Running Tests
Tests are located in the `testing_framework` directory. To run the tests, use `pytest`:
```bash
pytest testing_framework/
```

## 4. Contributing
1. Create a new branch for your feature or bug fix.
2. Make your changes.
3. Add or update tests for your changes.
4. Ensure all tests pass.
5. Create a pull request.
