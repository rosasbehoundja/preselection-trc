from typing import Tuple
from abc import abstractmethod, ABC
import math

class Robot(ABC):
    """
    Abstract base class for all robot types.
    This class defines the basic properties and methods that all robot types should implement.
    """

    MAX_BATTERY = 100
    MIN_BATTERY = 0
    LOW_BATTERY_LEVEL = 15

    def __init__(
            self,
            name: str = "LeRobot",
            x: float = 0.0,
            y: float = 0.0,
            battery: int = 100,
            is_active: bool = True
    ) -> None:
        """
        Initialize a Robot instance.

        Args:
            name (str): Robot's name
            x (float): Initial X position
            y (float): Initial Y position
            battery (int): Initial charge level
            is_active (bool): State of the robot (on/off)

        Raises:
            ValueError: if invalid parameters
        """

        if not name or not isinstance(name, str):
            raise ValueError("Name must be string")
            return
        if (battery < self.MIN_BATTERY or battery > self.MAX_BATTERY):
            raise ValueError(f"Battery level must be between {self.MIN_BATTERY} and {self.MAX_BATTERY}")
        
        self._name = name
        self._x = x
        self._y = y
        self._battery_level = battery
        self._is_active = is_active

    @property
    def name(self) -> str:
        return self._name
    
    @property
    def battery_level(self) -> int:
        return self._battery_level
    
    @battery_level.setter
    def battery_level(self, value: int = 100) -> None:
        """Charge battery"""
        if value < self.MIN_BATTERY or value > self.MAX_BATTERY:
            raise ValueError(f"Battery level must be between {self.MIN_BATTERY} and {self.MAX_BATTERY}")
        self._battery_level = value
    
    @abstractmethod
    def move(self, target_x: float = 0.0, target_y: float = 0.0) -> bool:
        """
        Abstract method to move the robot.
        
        Args:
            target_x (float): Target X coordinate
            target_y (float): Target Y coordinate

        Returns:
            bool: True if robot moves successfully.
        """
        pass

    @abstractmethod
    def position(self) -> Tuple:
        """Get the current position of the robot."""
        return (self._x, self._y)

    @abstractmethod
    def get_distance_to(self, target_x: float, target_y: float) -> float:
        """Compute the distance between actual robot position and target position"""
        return math.sqrt((self._x - target_x)**2 + (self._y - target_y)**2)

    def is_active(self) -> bool:
        """Return True if robot still got active"""
        return self.battery_level > 0 and self._is_active
    
    def is_battery_low(self) -> bool:
        return self.battery_level <= self.LOW_BATTERY_LEVEL

    def __str__(self) -> str:
        """Returns a string representation of the Robot instance."""
        return f"Robot(name={self.name}, position={self.position}, battery_level={self.battery_level})"
