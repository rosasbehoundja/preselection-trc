from .robot import Robot


class WheeledRobot(Robot):
    """
    Type of robot that moves on wheels.
    """
    def __init__(
            self,
            name: str = "WheeledRobot",
            x: float = 0.0,
            y: float = 0.0,
            battery: int = 100,
            is_active: bool = True,
            max_speed: float = 10.0,
    ) -> None:
        """
        Initializes a WheeledRobot instance.

        Args:
            name (str): Robot's name
            x (float): Initial X position
            y (float): Initial Y position
            battery (int): Initial charge level
            is_active (bool): State of the robot (on/off)
            max_speed (float): Maximum speed of the robot

        Raises:
            ValueError: if invalid parameters
        """

        if not name or not isinstance(name, str):
            raise ValueError("Name must be string")
        if (battery < self.MIN_BATTERY or battery > self.MAX_BATTERY):
            raise ValueError(f"Battery level must be between {self.MIN_BATTERY} and {self.MAX_BATTERY}")
        if not isinstance(max_speed, (int, float)) or max_speed <= 0:
            raise ValueError("max_speed must be a number.")
        
        super().__init__(name, x, y, battery, is_active)
        self._max_speed = max_speed
        self._current_speed: float = 0.0
        self._direction: float = 0.0 # direction in degrees

    @property
    def max_speed(self) -> float:
        return self._max_speed
    
    @property
    def current_speed(self) -> float:
        return self._current_speed
    
    @property
    def direction(self) -> float:
        return self._direction
    
    def set_speed(self, speed: float) -> None:
        """Set the current speed of the robot."""
        if not isinstance(speed, (int, float)):
            raise ValueError("Speed must be a number.")
        if speed < 0 or speed > self.max_speed:
            raise ValueError(f"Speed must be between 0 and {self.max_speed}.")
        
        self._current_speed = speed

    def set_direction(self, direction: float) -> None:
        """Set the direction of the robot."""
        if not isinstance(direction, (int, float)):
            raise ValueError("Direction must be a number.")
        if direction < 0 or direction >= 360:
            raise ValueError("Direction must be between 0 and 360 degrees.")
        
        self._direction = direction

    def move(self, target_x: float = 0.0, target_y: float = 0.0) -> bool:
        """
        Move the robot towards a target position.

        Args:
            target_x (float): Target X position
            target_y (float): Target Y position
        Returns:
            bool: True if the robot moved successfully, False otherwise.
        """
        if not self.is_active:
            print(f"{self.name} is not active. Cannot move.")
            return False
        self._x = target_x
        self._y = target_y
        return True
