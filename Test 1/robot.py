from abc import ABC, abstractmethod
from typing import Union, Tuple, Any, List
import logging
import math


class Robot(ABC):
    """
    Abstract base class representing a robot in a simulation environment.
    """
    # Class variable to define valid energy sources
    ENERGY_SOURCE = ["solar", "fossil_fuel", "electric"]

    def  __init__(
            self,
            id: Union[int, str],
            name: str,
            position: Tuple[float, float],
            orientation: float,
            energy_source: str,
        ) -> None:
        """
        Initialize a Robot instance.
        
        Args:
            id (Union[int, str]): Unique identifier for the robot.
            name (str): Name of the robot.
            position (Tuple[float, float]): The (x, y) coordinates of the robot's position.
            orientation (float): The orientation of the robot in degrees.
            energy_source (str): The energy source used by the robot, must be one of the ENERGY_SOURCE list.

        ENERGY_SOURCE is a list of valid energy sources:
            - "solar"
            - "fossil_fuel"
            - "electric"

        Raises:
            TypeError: If any of the parameters are of incorrect type.
            ValueError: If the energy source is not in the ENERGY_SOURCE list.
        """
        # Validate input types
        if not isinstance(id, (int, str)):
            raise TypeError("ID must be an integer or a string.")
        if not isinstance(name, str):
            raise TypeError("Name must be a string.")
        if not isinstance(position, tuple) or len(position) != 2 or not all(isinstance(coord, (int, float)) for coord in position):
            raise TypeError("Position must be a tuple of two numeric values (x, y).")
        if not isinstance(orientation, (int, float)):
            raise TypeError("Orientation must be a numeric value (in radians).")
        if energy_source.lower() not in self.ENERGY_SOURCE:
            raise ValueError("Energy source must be an instance of ENERGY_SOURCE List.")
        
        self._id = id
        self._name = name
        self._position = position
        self._orientation = orientation
        self._energy_source = energy_source

        # Indicates if the robot is currently active
        self._is_active = True

        # Generator power level
        self._generator_level = 100

        # List to hold sensors attached to the robot
        self._sensors: List[Any] = []

        # Setup logging
        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self._logger.addHandler(handler)
    
    # Properties to access robot attributes
    @property
    def id(self) -> Union[int, str]:
        """Get the unique identifier of the robot."""
        return self._id
    
    @property
    def name(self) -> str:
        """Get the name of the robot."""
        return self._name
    
    @property
    def position(self) -> Tuple[float, float]:
        """Get the current position of the robot."""
        return self._position
    
    @property
    def orientation(self) -> float:
        """Get the current orientation of the robot in radians."""
        return self._orientation
    
    @property
    def energy_source(self) -> str:
        """Get the energy source of the robot."""
        return self._energy_source
    
    @property
    def generator_level(self) -> int:
        """Get the current generator level of the robot."""
        return self._generator_level
    
    @property
    def is_active(self) -> bool:
        """Check if the robot is currently active."""
        return self._is_active
    
    @property
    def sensors(self) -> List[Any]:
        """Get the list of sensors attached to the robot."""
        return self._sensors
    
    # Setters to modify robot attributes
    @position.setter
    def position(self, new_position: Tuple[float, float]) -> None:
        """Set a new position for the robot."""
        if not isinstance(new_position, tuple) or len(new_position) != 2 or not all(isinstance(coord, (int, float)) for coord in new_position):
            raise TypeError("Position must be a tuple of two numeric values (x, y).")
        self._position = new_position
        self._logger.info(f"Position updated to {self._position}")
    
    @orientation.setter
    def orientation(self, new_orientation: float) -> None:
        """Set a new orientation for the robot."""
        if not isinstance(new_orientation, (int, float)):
            raise TypeError("Orientation must be numeric (degrees).")
        
        # Make sure the orientation is within [0, 360) degrees
        if new_orientation < 0 or new_orientation >= 360:
            new_orientation = new_orientation % 360
        # Normalize orientation to be within [0, 2π)
        radian_orientation = math.radians(new_orientation)
        self._orientation = math.fmod(radian_orientation, 2 * math.pi)
        self._logger.info(f"Orientation updated to {self._orientation} radians")

    @generator_level.setter
    def generator_level(self, new_level: int) -> None:
        """Set a new generator level for the robot."""
        if not isinstance(new_level, int):
            raise TypeError("Generator level must be an integer.")
        self._generator_level = int(new_level)
        self._logger.info(f"Generator level updated to {self._generator_level}")

    @is_active.setter
    def is_active(self, active: bool) -> None:
        """Set the active status of the robot."""
        if self._generator_level <= 0:
            self._logger.warning("Generator level is zero, cannot set active status.")
            return
        if not isinstance(active, bool):
            raise TypeError("is_active must be a boolean.")
        if active and not self._is_active:
            self._logger.info("Robot activated.")
        elif not active and self._is_active:
            self._logger.info("Robot deactivated.")

        self._is_active = active
    
    # Abstract methods to be implemented by subclasses
    @abstractmethod
    def move(self, direction: str, distance: float) -> None:
        """
        Move the robot in a specified direction by a certain distance.
        
        Args:
            direction (str): The direction to move ('forward', 'backward', 'left', 'right').
            distance (float): The distance to move in the specified direction.
        """
        raise NotImplementedError

    @abstractmethod
    def rotate(self, angle: float) -> None:
        """
        Rotate the robot by a certain angle.

        Args:
            angle (float): The angle to rotate the robot, in radians.
        """
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the robot's movement.
        """
        raise NotImplementedError

    @abstractmethod
    def recharge(self) -> None:
        """
        Recharge the robot's energy source.
        """
        raise NotImplementedError

    @abstractmethod
    def status(self) -> str:
        """
        Get the current status of the robot.
        
        Returns:
            str: A string representation of the robot's current status.
        """
        return (f"Robot(ID: {self.id}, Name: {self.name}, Position: {self.position}, "
                f"Orientation: {self.orientation}, Energy Source: {self.energy_source}, "
                f"Active: {self.is_active})")
    
    @abstractmethod
    def distance_to(self, *args: float) -> float:
        """
        Calculate the distance from the robot to another point.

        Args:
            other (Tuple[float, float]): The (x, y) coordinates of the other point.

        Returns:
            float: The distance to the other point.
        """
        x0, y0 = self.position
        x1, y1 = args
        return math.hypot(x1 - x0, y1 - y0)

    # Personalized methods for Robot
    def consume_energy(self, amount: float) -> None:
        """Consume energy from the robot's battery."""
        if not self._is_active:
            self._logger.warning("Robot is inactive, cannot consume energy.")
            return
        if amount < 0:
            raise ValueError("Energy consumption must be positive.")
        self._battery_level -= amount
        self._logger.info(f"Consumed {amount} energy. Battery level is now {self._battery_level}.")

    def recharge(self, amount: float) -> None:
        """
        Recharge the robot's battery.
        
        Args:
            amount (float): The amount of energy to recharge.
        """
        if not self._is_active:
            self._logger.warning("Robot is inactive, cannot recharge.")
            return
        if amount < 0:
            raise ValueError("Recharge amount must be positive.")
        self._battery_level += amount
        if self._battery_level > 100:
            self._battery_level = 100
        self._logger.info(f"Recharged {amount} energy. Battery level is now {self._battery_level}.")
        if self._battery_level <= 0:
            self._is_active = False
            self._logger.warning("Battery depleted: robot deactivated.")
        self._logger.info(f"Robot {self.name} is now {'active' if self.is_active else 'inactive'}.")

    def add_sensor(self, sensor: Any) -> None:
        """
        Add a sensor to the robot.
        
        Args:
            sensor (Any): The sensor to add.
        """
        if sensor not in self._sensors:
            self._sensors.append(sensor)
            self._logger.info(f"Sensor {sensor} added to robot {self.name}.")
        else:
            self._logger.warning(f"Sensor {sensor} is already attached to robot {self.name}.")

    def remove_sensor(self, sensor: Any) -> None:
        """
        Remove a sensor from the robot.
        
        Args:
            sensor (Any): The sensor to remove.
        """
        if sensor in self._sensors:
            self._sensors.remove(sensor)
            self._logger.info(f"Sensor {sensor} removed from robot {self.name}.")
        else:
            self._logger.warning(f"Sensor {sensor} not found on robot {self.name}.")

    def __str__(self) -> str:
        """
        Return a string representation of the robot.
        
        Returns:
            str: A string containing the robot's ID, name, position, orientation, energy source, and active status.
        """
        return self.status()
    
    def __repr__(self) -> str:
        """
        Return a detailed string representation of the robot for debugging.
        
        Returns:
            str: A string containing the class name and the robot's attributes.
        """
        return (f"{self.__class__.__name__}(ID={self.id}, Name={self.name}, "
                f"Position={self.position}, Orientation={self.orientation}, "
                f"Energy Source={self.energy_source}, Active={self.is_active})")
    
    def __eq__(self, other: object) -> bool:
        """
        Check if two robots are equal based on their ID.
        
        Args:
            other (object): The object to compare with.
        
        Returns:
            bool: True if the IDs are the same, False otherwise.
        """
        if not isinstance(other, Robot):
            return NotImplemented
        return self.id == other.id
    
    