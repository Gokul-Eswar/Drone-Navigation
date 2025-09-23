from abc import ABC, abstractmethod
from typing import Dict, Any
from dataclasses import dataclass, field

@dataclass
class DroneState:
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # quaternion
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_level: float = 0.0
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    connected: bool = False

@dataclass
class DroneCapabilities:
    max_velocity: float = 5.0
    max_acceleration: float = 2.0
    max_altitude: float = 50.0
    payload_capacity: float = 1.0
    flight_time: float = 20.0
    has_gimbal: bool = False
    camera_specs: Dict[str, Any] = field(default_factory=dict)

class UniversalDroneInterface(ABC):
    """Base interface that all drone types inherit from"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.state = DroneState()
        self.capabilities = self._load_capabilities()
        self._safety_limits = self._load_safety_limits()

    @abstractmethod
    async def connect(self) -> bool:
        """Establish connection to drone"""
        pass

    @abstractmethod
    async def arm(self) -> bool:
        """Arm the drone motors"""
        pass

    @abstractmethod
    async def takeoff(self, altitude: float = 1.5) -> bool:
        """Execute takeoff sequence"""
        pass

    @abstractmethod
    async def goto_position(self, x: float, y: float, z: float, yaw: float = 0) -> bool:
        """Navigate to specific position"""
        pass

    @abstractmethod
    async def land(self) -> bool:
        """Execute landing sequence"""
        pass

    @abstractmethod
    async def emergency_stop(self) -> bool:
        """Immediate emergency stop"""
        pass

    def get_state(self) -> DroneState:
        """Get current drone state"""
        return self.state

    def get_capabilities(self) -> DroneCapabilities:
        """Get drone capabilities"""
        return self.capabilities

    def _load_capabilities(self) -> DroneCapabilities:
        """Load drone capabilities from config"""
        print("Loading capabilities...")
        # In a real implementation, this would parse self.config
        return DroneCapabilities()

    def _load_safety_limits(self) -> Dict:
        """Load safety limits from config"""
        print("Loading safety limits...")
        # In a real implementation, this would parse self.config
        return {}
