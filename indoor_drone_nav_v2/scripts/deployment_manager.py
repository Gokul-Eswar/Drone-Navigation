import yaml
import os
import subprocess
import argparse
from typing import Dict, List, Optional
from pathlib import Path

class BaseDeployment:
    """Base class for platform-specific deployments."""
    def deploy(self, config_file: str):
        raise NotImplementedError

    def _run_command(self, command, **kwargs):
        print(f"Mock Executing: {' '.join(command)}")
        # In a real deployment, you would run this. For safety in this environment, we mock.
        # try:
        #     subprocess.run(command, check=True, capture_output=True, text=True, **kwargs)
        # except subprocess.CalledProcessError as e:
        #     print(f"Error executing command: {e}\nSTDOUT: {e.stdout}\nSTDERR: {e.stderr}")
        # except FileNotFoundError:
        #      print(f"Error: Command '{command[0]}' not found.")

    def _build_docker_image(self, tag: str, dockerfile_path: str):
        print(f"Building Docker image {tag} from {dockerfile_path}...")
        self._run_command(['docker', 'build', '-t', tag, '-f', dockerfile_path, '.'])

    def _deploy_with_constraints(self, constraints: Dict):
        print(f"Deploying with constraints: {constraints} (mock)...")


class JetsonNanoDeployment(BaseDeployment):
    """Deployment for NVIDIA Jetson Nano"""

    def deploy(self, config_file: str):
        print("\n🚀 Deploying to Jetson Nano...")
        self._install_nvidia_docker()
        self._optimize_for_jetson_nano()

        dockerfile_path = Path("Docker/Dockerfile.jetson_nano")
        if not dockerfile_path.exists():
            print(f"⚠️ Warning: Dockerfile not found at {dockerfile_path}. Skipping build.")
            return

        self._build_docker_image("indoor_drone_nav:jetson_nano", str(dockerfile_path))

        self._deploy_with_constraints({
            'memory': '3GB', 'cpu_count': 4, 'gpu_memory': '512MB'
        })
        print("✅ Jetson Nano deployment complete.")

    def _install_nvidia_docker(self):
        print("Installing NVIDIA container runtime (mock)...")

    def _optimize_for_jetson_nano(self):
        """Optimize algorithms for Jetson Nano's limited resources"""
        print("Optimizing configuration for Jetson Nano...")
        optimizations = {
            'slam': {'algorithm': 'lightweight_cartographer', 'map_resolution': 0.1, 'update_frequency': 5},
            'computer_vision': {'model': 'mobilenet_v2', 'input_resolution': [320, 240], 'inference_threads': 2},
            'navigation': {'planner': 'rrt_star_lite', 'planning_frequency': 2}
        }

        opt_dir = Path('config/platform_optimizations')
        opt_dir.mkdir(exist_ok=True)
        opt_file = opt_dir / 'jetson_nano.yaml'
        with open(opt_file, 'w') as f:
            yaml.dump(optimizations, f)
        print(f"  - Saved optimizations to {opt_file}")


class JetsonXavierDeployment(BaseDeployment):
    """Deployment for NVIDIA Jetson Xavier"""
    def deploy(self, config_file: str):
        print("\n🚀 Deploying to Jetson Xavier (placeholder)...")
        print("✅ Jetson Xavier deployment complete.")

class RaspberryPiDeployment(BaseDeployment):
    """Deployment for Raspberry Pi"""
    def deploy(self, config_file: str):
        print("\n🚀 Deploying to Raspberry Pi (placeholder)...")
        print("✅ Raspberry Pi deployment complete.")

class DesktopDeployment(BaseDeployment):
    """Deployment for a standard desktop"""
    def deploy(self, config_file: str):
        print("\n🚀 Deploying to Desktop...")
        print("  - No special optimizations needed.")
        dockerfile_path = Path("Docker/Dockerfile.desktop")
        if not dockerfile_path.exists():
            print(f"⚠️ Warning: Dockerfile not found at {dockerfile_path}. Skipping build.")
            return
        self._build_docker_image("indoor_drone_nav:desktop", str(dockerfile_path))
        print("✅ Desktop deployment complete.")

class SimulationDeployment(BaseDeployment):
    """Deployment for simulation"""
    def deploy(self, config_file: str):
        print("\n🚀 Deploying for Simulation...")
        print("  - Using simulation-specific configurations.")
        dockerfile_path = Path("Docker/Dockerfile.simulation")
        if not dockerfile_path.exists():
            print(f"⚠️ Warning: Dockerfile not found at {dockerfile_path}. Skipping build.")
            return
        self._build_docker_image("indoor_drone_nav:simulation", str(dockerfile_path))
        print("✅ Simulation deployment complete.")


class DeploymentManager:
    """Manages deployment across different environments"""

    def __init__(self):
        self.supported_platforms = {
            'jetson_nano': JetsonNanoDeployment(),
            'jetson_xavier': JetsonXavierDeployment(),
            'raspberry_pi': RaspberryPiDeployment(),
            'desktop': DesktopDeployment(),
            'simulation': SimulationDeployment()
        }

    def deploy(self, platform: str, config_file: str):
        """Deploy system to specified platform"""
        if platform not in self.supported_platforms:
            raise ValueError(f"Unsupported platform: {platform}. Supported: {list(self.supported_platforms.keys())}")

        deployment_handler = self.supported_platforms[platform]
        deployment_handler.deploy(config_file)

    def generate_deployment_config(self, platform: str,
                                 drone_type: str,
                                 sensors: List[str]) -> str:
        """Generate optimal deployment configuration (placeholder)"""
        print(f"Generating deployment config for {platform} with {drone_type} and sensors: {sensors}...")
        config = {
            'platform': platform,
            'drone': {'type': drone_type},
            'sensors': sensors,
            'docker': {'gpu_support': platform.startswith('jetson')}
        }

        config_filename = f"generated_config_{platform}.yaml"
        with open(config_filename, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)

        print(f"  - Saved generated config to {config_filename}")
        return config_filename

def main():
    parser = argparse.ArgumentParser(description='Deployment Manager for Indoor Drone Navigation System')
    parser.add_argument('platform', choices=['desktop', 'jetson_nano', 'jetson_xavier', 'raspberry_pi', 'simulation'],
                        help='The target platform to deploy to.')
    parser.add_argument('--config', default='config/drones/active.yaml',
                        help='Path to the active drone configuration file.')

    args = parser.parse_args()

    print(f"--- Deployment Manager ---")
    if not Path("scripts").is_dir():
         print("❌ Error: This script must be run from the root of the 'indoor_drone_nav_v2' project.")
         return 1

    manager = DeploymentManager()
    manager.deploy(args.platform, args.config)
    print(f"--- Deployment Finished ---")

if __name__ == '__main__':
    if Path.cwd().name == 'scripts':
        os.chdir('..')

    # Create dummy files for testing
    Path('Docker').mkdir(exist_ok=True)
    Path('Docker/Dockerfile.jetson_nano').touch()
    Path('Docker/Dockerfile.desktop').touch()
    Path('Docker/Dockerfile.simulation').touch()

    # Example usage:
    # python scripts/deployment_manager.py jetson_nano
    # main() # Ccommented out to prevent execution in this environment without args
