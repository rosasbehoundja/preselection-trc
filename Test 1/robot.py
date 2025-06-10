from abc import ABC, abstractmethod
from typing import Union, Tuple

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
            orientation (float): The orientation of the robot in radians.
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
    
    # Setters to modify robot attributes
    @position.setter
    def position(self, new_position: Tuple[float, float]) -> None:
        """Set a new position for the robot."""
        self._position = new_position
    
    @orientation.setter
    def orientation(self, new_orientation: float) -> None:
        """Set a new orientation for the robot."""
        self._orientation = new_orientation

    @generator_level.setter
    def generator_level(self, new_level: int) -> None:
        """Set a new generator level for the robot."""
        self._generator_level = new_level

    @is_active.setter
    def is_active(self, active: bool) -> None:
        """Set the active status of the robot."""
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
        pass

    @abstractmethod
    def rotate(self, angle: float) -> None:
        """
        Rotate the robot by a certain angle.

        Args:
            angle (float): The angle to rotate the robot, in radians.
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """
        Stop the robot's movement.
        """
        pass

    @abstractmethod
    def recharge(self) -> None:
        """
        Recharge the robot's energy source.
        """
        pass

    @abstractmethod
    def status(self) -> str:
        """
        Get the current status of the robot.
        
        Returns:
            str: A string representation of the robot's current status.
        """
        pass

    def __str__(self) -> str:
        """
        Return a string representation of the robot.
        
        Returns:
            str: A string containing the robot's ID, name, position, orientation, energy source, and active status.
        """
        return (f"Robot(ID: {self.id}, Name: {self.name}, Position: {self.position}, "
                f"Orientation: {self.orientation}, Energy Source: {self.energy_source}, "
                f"Active: {self.is_active})")
    
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
    
    