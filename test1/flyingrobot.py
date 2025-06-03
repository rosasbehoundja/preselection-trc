from .robot import Robot
from typing import Tuple


class FlyingRobot(Robot):
    """
    Type of robot that can fly.
    """

    def __init__(
            self, 
            name: str = "FlyingRobot", 
            x: float = 0.0, 
            y: float = 0.0, 
            battery: int = 100, 
            is_active: bool = True,
            max_altitude: float = 100.0
        ) -> None:
        """
        Initializes a FlyingRobot instance.
        
        Args:
            name (str): Robot's name
            x (float): Initial X position
            y (float): Initial Y position
            battery (int): Initial charge level
            is_active (bool): State of the robot (on/off)
            max_altitude (float): Maximum altitude the robot can reach

        Raises:
            ValueError: if invalid parameters
        """

        if not name or not isinstance(name, str):
            raise ValueError("Name must be string")
        if (battery < self.MIN_BATTERY or battery > self.MAX_BATTERY):
            raise ValueError(f"Battery level must be between {self.MIN_BATTERY} and {self.MAX_BATTERY}")
        if not isinstance(max_altitude, (int, float)) or max_altitude <= 0:
            raise ValueError("max_altitude must be a number.")
        
        super().__init__(name, x, y, battery, is_active)
        self._max_altitude = max_altitude
        self._current_altitude: float = 0.0
        self._is_flying: bool = False
     
    @property
    def max_altitude(self) -> float:
        return self._max_altitude
    
    @property
    def current_altitude(self) -> float:
        return self._current_altitude
    
    @property
    def is_flying(self) -> bool:
        return self.current_altitude > 0
    
    def position(self) -> Tuple[float, float, float]:
        return (*super().position(), self.current_altitude)
    
    def take_off(self, altitude: float) -> None:
        pass

    def land(self) -> None:
        pass

    def move(self, target_x: float = 0.0, target_y: float = 0.0, target_altitude: float = 0.0) -> bool:
        """
        Move the robot towards a target position.
        
        Args:
            target_x (float): Target X position
            target_y (float): Target Y position
            target_altitude (float): Target altitude
        Returns:
            bool: True if the robot moved successfully, False otherwise.
        """
        if not self.is_active:
            print(f"{self.name} is not active. Cannot move.")
            return False
        self._x = target_x
        self._y = target_y
        self._current_altitude = target_altitude
        
        return True
