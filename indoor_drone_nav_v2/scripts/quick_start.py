#!/usr/bin/env python3
"""
One-command setup and launch for Indoor Drone Navigation System V2

Usage:
    python scripts/quick_start.py --setup px4_basic
    python scripts/quick_start.py --setup dji_matrice --platform jetson_nano
    python scripts/quick_start.py --simulation --test
"""

import argparse
import subprocess
import sys
import os
from pathlib import Path

# Assuming deployment_manager.py is in the same directory
try:
    from deployment_manager import DeploymentManager
except ImportError:
    print("Warning: Could not import DeploymentManager. Ensure deployment_manager.py is in the scripts directory.")
    class DeploymentManager:
        def deploy(self, platform, config):
            print(f"Mock DeploymentManager: Deploying to {platform} with {config}")

class ComprehensiveSetupSystem:
    """One-command setup for any configuration"""

    def __init__(self):
        self.supported_setups = {
            'px4_basic': self._setup_px4_basic,
            'px4_advanced': self._setup_px4_advanced,
            'dji_matrice': self._setup_dji_matrice,
            'simulation': self._setup_simulation,
            'research_platform': self._setup_research_platform
        }

    def setup(self, setup_type: str, **kwargs):
        """One-command setup"""
        if setup_type not in self.supported_setups:
            print(f"❌ Unknown setup type: {setup_type}")
            print(f"Available setups: {list(self.supported_setups.keys())}")
            return False

        print(f"🚀 Setting up {setup_type} configuration...")
        return self.supported_setups[setup_type](**kwargs)

    def _run_command(self, command, **kwargs):
        print(f"Executing: {' '.join(command)}")
        try:
            # Using capture_output to prevent polluting stdout, but printing if error occurs
            result = subprocess.run(command, check=True, capture_output=True, text=True, **kwargs)
            return result
        except FileNotFoundError:
            print(f"❌ Error: Command '{command[0]}' not found.")
        except subprocess.CalledProcessError as e:
            print(f"❌ Error executing command: {' '.join(command)}")
            print(f"  - Return Code: {e.returncode}")
            print(f"  - STDOUT: {e.stdout}")
            print(f"  - STDERR: {e.stderr}")
        return None

    def _install_ros2_packages(self, packages: list):
        print(f"Mock installing ROS2 packages: {packages}")
        # In a real scenario, this would use sudo and might need user password.
        # Example: self._run_command(["sudo", "apt-get", "install", "-y"] + packages)

    def _configure_mavros(self):
        print("Configuring MAVROS (placeholder)...")

    def _build_system(self):
        print("Building the system with colcon (mock)...")
        # self._run_command(["colcon", "build"])

    def _install_dji_sdk(self):
        print("Installing DJI SDK (placeholder)...")

    def _setup_px4_basic(self, **kwargs):
        """Setup for basic PX4 drone with camera"""
        print("📋 Setting up basic PX4 configuration...")

        self._install_ros2_packages([
            'ros-humble-mavros', 'ros-humble-mavros-extras',
            'ros-humble-cartographer-ros', 'ros-humble-nav2-bringup'
        ])
        self._configure_mavros()

        configs = {
            'drones': 'px4_quadcopter_basic.yaml', 'sensors': 'camera_imu_basic.yaml',
            'safety': 'standard_safety.yaml', 'navigation': 'basic_navigation.yaml'
        }

        print("Copying configuration files...")
        for config_type, filename in configs.items():
            src = Path(f'config/presets/{filename}')
            dst_dir = Path(f'config/{config_type}')
            dst_file = dst_dir / 'active.yaml'
            dst_dir.mkdir(parents=True, exist_ok=True)

            if src.exists():
                self._run_command(['cp', str(src), str(dst_file)])
                print(f"  - Copied {src} to {dst_file}")
            else:
                print(f"  - ⚠️ Warning: Preset file not found: {src}. Creating empty file at {dst_file}.")
                dst_file.touch()

        self._build_system()

        print("\n✅ Basic PX4 setup complete!")
        print("🚁 Connect your drone and run: ros2 launch indoor_drone_nav_v2 px4_basic.launch.py")
        return True

    def _setup_px4_advanced(self, **kwargs):
        print("📋 Setting up advanced PX4 configuration (placeholder)...")
        return True

    def _setup_dji_matrice(self, **kwargs):
        """Setup for DJI Matrice with professional sensors"""
        print("📋 Setting up DJI Matrice configuration...")
        self._install_dji_sdk()

        configs = {
            'drones': 'dji_matrice_300.yaml', 'sensors': 'lidar_camera_professional.yaml',
            'safety': 'commercial_safety.yaml', 'navigation': 'professional_navigation.yaml'
        }

        print("Copying configuration files...")
        for config_type, filename in configs.items():
            src = Path(f'config/presets/{filename}')
            dst_dir = Path(f'config/{config_type}')
            dst_file = dst_dir / 'active.yaml'
            dst_dir.mkdir(parents=True, exist_ok=True)
            if src.exists():
                self._run_command(['cp', str(src), str(dst_file)])
                print(f"  - Copied {src} to {dst_file}")
            else:
                print(f"  - ⚠️ Warning: Preset file not found: {src}. Creating empty file at {dst_file}.")
                dst_file.touch()

        self._build_system()

        print("\n✅ DJI Matrice setup complete!")
        print("📱 Configure DJI SDK credentials in: config/drones/active.yaml")
        return True

    def _setup_simulation(self, **kwargs):
        print("📋 Setting up simulation configuration (placeholder)...")
        return True

    def _setup_research_platform(self, **kwargs):
        print("📋 Setting up research platform configuration (placeholder)...")
        return True


def main():
    parser = argparse.ArgumentParser(description='Quick Start for Indoor Drone Navigation System V2')
    parser.add_argument('--setup', choices=['px4_basic', 'px4_advanced', 'dji_matrice', 'simulation', 'research_platform'], help='The type of setup to configure.')
    parser.add_argument('--platform', choices=['desktop', 'jetson_nano', 'jetson_xavier', 'raspberry_pi'], default='desktop', help='The deployment platform.')
    parser.add_argument('--simulation', action='store_true', help='Use simulation mode for launching.')
    parser.add_argument('--test', action='store_true', help='Run tests after setup.')
    parser.add_argument('--gui', action='store_true', help='Launch the GUI interface along with the system.')

    args = parser.parse_args()

    print("🚁 Indoor Drone Navigation System V2 - Quick Start 🚁")
    print("======================================================")

    if not Path("scripts").is_dir():
        print("❌ Error: This script must be run from the root of the 'indoor_drone_nav_v2' project.")
        return 1

    if not check_prerequisites():
        print("⚠️ Warning: Prerequisites not fully met. Functionality may be limited.")

    if args.setup:
        setup_system = ComprehensiveSetupSystem()
        if not setup_system.setup(args.setup, platform=args.platform):
            print("❌ System setup failed.")
            return 1
        print("✅ System setup completed successfully.")

    if args.platform != 'desktop':
        print(f"\n🚀 Deploying to platform: {args.platform}...")
        deployment_manager = DeploymentManager()
        active_config = 'config/drones/active.yaml'
        if Path(active_config).exists():
            deployment_manager.deploy(args.platform, active_config)
        else:
            print(f"⚠️ Warning: Active config file not found at {active_config}. Skipping deployment.")

    if args.test:
        print("\n🧪 Running system tests...")
        result = subprocess.run(['python3', '-m', 'pytest', 'testing_framework/'], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"❌ Tests failed!\n--- STDOUT ---\n{result.stdout}\n--- STDERR ---\n{result.stderr}")
        else:
            print(f"✅ All tests passed!\n--- STDOUT ---\n{result.stdout}")

    print("\n🚀 Launching the system...")
    launch_system(args)

    print("\n🎉 Quick Start process finished.")
    return 0

def check_prerequisites():
    """Check if all prerequisites are installed"""
    print("Checking prerequisites...")
    required = {'ros2', 'colcon', 'docker'}
    found = set()

    for cmd in required:
        if subprocess.run(['which', cmd], capture_output=True).returncode == 0:
            print(f"  - ✅ {cmd} found.")
            found.add(cmd)
        else:
            print(f"  - ❌ {cmd} not found. Please install it.")

    return found == required

def launch_system(args):
    """Launch the navigation system using ROS2 launch"""
    print("Constructing launch command...")
    launch_file = f"{args.setup if args.setup else 'master'}.launch.py"
    if args.simulation:
        launch_file = 'simulation.launch.py'

    launch_cmd = ['ros2', 'launch', 'indoor_drone_nav_v2', launch_file]
    if args.gui:
        launch_cmd.append('gui:=true')

    print(f"▶️  Mock Executing: {' '.join(launch_cmd)}")
    print("(In a real scenario, the ROS2 system would start now.)")

if __name__ == '__main__':
    # Ensure the script is run from the project root.
    # If it's run from 'scripts/', change up one directory.
    if Path.cwd().name == 'scripts':
        os.chdir('..')
        print("Changed directory to project root.")

    sys.exit(main())
