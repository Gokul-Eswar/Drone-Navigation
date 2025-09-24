import asyncio
from .universal_interface import UniversalDroneInterface, DroneState

class SimulationInterface(UniversalDroneInterface):
    """Gazebo/simulation implementation"""

    async def connect(self) -> bool:
        # Simulation connection logic
        print("Connecting to simulation...")
        await asyncio.sleep(0.1)
        self.state.connected = True
        print("Simulation connected.")
        return True

    async def arm(self) -> bool:
        print("Arming in simulation...")
        self.state.armed = True
        return True

    async def takeoff(self, altitude: float = 1.5) -> bool:
        print(f"Taking off to {altitude}m in simulation...")
        if self.state.armed:
            self.state.position = (self.state.position[0], self.state.position[1], altitude)
            return True
        return False

    async def goto_position(self, x: float, y: float, z: float, yaw: float = 0) -> bool:
        print(f"Going to ({x}, {y}, {z}) with yaw {yaw} in simulation...")
        if self.state.armed:
            self.state.position = (x, y, z)
            return True
        return False

    async def land(self) -> bool:
        print("Landing in simulation...")
        self.state.position = (self.state.position[0], self.state.position[1], 0)
        self.state.armed = False
        return True

    async def emergency_stop(self) -> bool:
        print("EMERGENCY STOP in simulation!")
        self.state.armed = False
        return True
