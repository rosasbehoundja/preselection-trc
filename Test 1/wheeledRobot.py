from robot import Robot

from typing import Tuple, List, Any
from enum import Enum
import math


class State(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    UPDATING_STORAGE = "UPDATING_STORAGE"
    RETURNING = "RETURNING"
    CHARGING = "CHARGING"
    SHUTDOWN = "SHUTDOWN"


class WheeledRobot(Robot):
    """
    A class representing a mobile robot with wheels, capable of navigation, obstacle avoidance,
    and interaction with a robotic arm for picking up and storing objects.
    """
    def __init__(
        self,
        name: str,
        position: Tuple[float, float],
        orientation: float,
        energy_source: str,
        wheel_base: float,
        storage_capacity: int,
    ) -> None:
        """
        Initialize the mobile robot with the given parameters.

        Args:
            name (str): Name of the robot.
            position (Tuple[float, float]): Initial position (x, y) in meters.
            orientation (float): Initial orientation in radians.
            energy_source (str): The energy source used by the robot, must be one of the ENERGY_SOURCE list.
            wheel_base (float): Distance between the wheels in meters.
            storage_capacity (int): Maximum number of items the robot can store.

        ENERGY_SOURCE is a list of valid energy sources:
            - "solar"
            - "fossil_fuel"
            - "electric"

        Raises:
            ValueError: If wheel_base is not positive and storage_capacity is not positive.
        """
        super().__init__(name=name, position=position, orientation=orientation, energy_source=energy_source)

        if not isinstance(wheel_base, (int, float)) or wheel_base <= 0:
            raise ValueError("wheel_base must be a positive number.")
        if not isinstance(storage_capacity, int) or storage_capacity <= 0:
            raise ValueError("storage_capacity must be a positive integer.")
        
        self._wheel_base = float(wheel_base)
        self._v_lin_cmd = 0.0 # Linear speed command (m/s)
        self._v_ang_cmd = 0.0 # Angular speed command (rad/s)

        self._stockage_capacity = storage_capacity
        self._storage_bag = []

        self._state = State.IDLE
        self._obstacle_threshold = 0.3 # Min distance to consider an obstacle (meters)

    # Properties for the mobile robot
    @property
    def wheel_base(self) -> float:
        """Get the wheel base of the robot."""
        return self._wheel_base
    
    @property
    def storage_capacity(self) -> int:
        """Get the storage capacity of the robot."""
        return self._stockage_capacity
    
    @property
    def state(self) -> State:
        """Get the current state of the robot."""
        return self._state
    
    @property
    def storage_bag(self) -> List[Any]:
        """Get the current storage bag."""
        return self._storage_bag.copy()
    
    @property
    def obstacle_threshold(self) -> float:
        """Get the obstacle detection threshold."""
        return self._obstacle_threshold
    
    @property
    def v_lin_cmd(self) -> float:
        """Get the current linear speed command."""
        return self._v_lin_cmd

    @property
    def v_ang_cmd(self) -> float:
        """Get the current angular speed command."""
        return self._v_ang_cmd
    
    @state.setter
    def state(self, new_state: State) -> None:
        """Set the current state of the robot."""
        if not isinstance(new_state, State):
            raise ValueError("state must be an instance of State Enum.")
        self._state = new_state
        self._logger.info(f"State changed to {self._state.value}")

    @obstacle_threshold.setter
    def obstacle_threshold(self, threshold: float) -> None:
        """Set the obstacle detection threshold."""
        if not isinstance(threshold, (int, float)) or threshold <= 0:
            raise ValueError("obstacle_threshold must be a positive number.")
        self._obstacle_threshold = float(threshold)
        self._logger.info(f"Obstacle threshold set to {self._obstacle_threshold:.2f} meters")
    
    # Personalized methods for WheeledRobot
    def set_motor_speed(self, left: float, right: float) -> None:
        """
        Define motors speeds for left and right wheels.

        Args:
            left (float): Speed for the left wheel (m/s).
            right (float): Speed for the right wheel (m/s).
        Raises:
            ValueError: If left or right speed is not a number.
        """
        if not isinstance(left, (int, float)):
            raise ValueError("left speed must be a number.")
        if not isinstance(right, (int, float)):
            raise ValueError("right speed must be a number.")

        self._v_lin_cmd = (left + right) / 2.0
        self._v_ang_cmd = (right - left) / self.wheel_base
        self._logger.info(f"Motors set: v={self._v_lin_cmd:.2f}, ω={self._v_ang_cmd:.2f}")

    def add_to_storage(self, item: Any) -> bool:
        """
        Add an item to the robot's storage bag.
        
        Args:
            item (Any): The item to be added.
        
        Returns:
            bool: True if the item was added, False if storage is full.
        """
        if len(self._storage_bag) >= self._stockage_capacity:
            self._logger.warning("Storage is full, cannot add item.")
            return False
        self.state = State.UPDATING_STORAGE
        self._storage_bag.append(item)
        self._logger.info(f"Item added to storage. Current count: {len(self._storage_bag)}")
        return True

    def move(self, direction: str, distance: float) -> None:
        if not self.is_active:
            self._logger.warning("Cannot move: Robot is inactive.")
            return
        if not isinstance(distance, (int, float)):
            raise ValueError("distance must be a number.")
        if distance < 0:
            raise ValueError("distance must be a non-negative number.")

        if direction not in ["forward", "backward"]:
            self._logger.error("Invalid move direction. Use 'forward', 'backward'")
            raise ValueError("move direction must be 'forward', 'backward'.")

        self.state = State.NAVIGATING
        angle = self.orientation if direction == "forward" else self.orientation + math.pi

        # Compute new position based on distance and angle
        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)
        new_position = (self.position[0] + dx, self.position[1] + dy)

        self._consume_for_motion(distance)
        self.position = new_position

        self._logger.info(f"Moved {direction} by {distance:.2f}m to {self.position}")

    def rotate(self, angle: float) -> None:
        if not self.is_active:
            self._logger.warning("Cannot rotate: Robot is inactive.")
            return

        self._consume_for_rotation(angle)
        self.orientation = angle

    def stop(self) -> None:
        if not self.is_active:
            self._logger.warning("Cannot stop: Robot is inactive.")
            return
        
        self._v_lin_cmd = 0.0
        self._v_ang_cmd = 0.0
        self.is_active = False
        self._logger.info("Robot stopped.")

    def detect_obstacle(self, obstacle_position: Tuple[float, float]) -> bool:
        """
        Check if an obstacle is detected within the threshold distance.
        """
        if not isinstance(obstacle_position, tuple) or len(obstacle_position) != 2:
            raise ValueError("obstacle_position must be a tuple (x, y).")
        
        distance = self.distance_to(obstacle_position)
        if distance < self._obstacle_threshold:
            self._logger.info(f"Obstacle detected at {obstacle_position} (distance: {distance:.2f}m)")
            return True
        return False

    def avoid_obstacle(self) -> bool:
        """Avoid an obstacle by rotating and moving around it."""
        if not self.is_active:
            self._logger.warning("Cannot avoid obstacle: Robot is inactive.")
            return False
        
        self.rotate(math.pi / 2)  # Rotate 90 degrees
        self.move("forward", 0.5)  # Move forward 0.5 meters
        self._logger.info("Obstacle avoided by rotating and moving forward.")
        return True

    def is_storage_full(self) -> bool:
        """Check if the storage bag is full."""
        full = len(self._storage_bag) >= self._stockage_capacity
        if full:
            self._logger.warning("Storage is full.")
        return full
    
    def status(self) -> str:
        """
        Get the current status of the robot.
        Returns:
            str: A string describing the robot's status.
        """
        status = (
            f"Robot '{self.name}' Status:\n"
            f"Position: {self.position}\n"
            f"Orientation: {self.orientation:.2f} radians\n"
            f"Energy Level: {self.generator_level:.2f}\n"
            f"State: {self.state.value}\n"
            f"Storage Capacity: {len(self._storage_bag)}/{self._stockage_capacity}\n"
            f"Linear Speed Command: {self._v_lin_cmd:.2f} m/s\n"
            f"Angular Speed Command: {self._v_ang_cmd:.2f} rad/s\n"
        )
        return status

    # Protected methods for internal use
    def _consume_for_motion(self, distance: float) -> None:
        """
        Consume energy proportionally to the distance moved.
        Args:
            distance (float): Distance moved in meters.
        Raises:
            ValueError: If distance is not a number or is not positive.
        """
        if not isinstance(distance, (int, float)):
            raise ValueError("distance must be a number.")
        if distance <= 0:
            self._logger.warning("Distance must be positive for energy consumption.")
            return
        
        amount = abs(distance) * 0.5 # 0.5% per meter
        self.consume_energy(amount)

    def _consume_for_rotation(self, angle: float) -> None:
        """
        Consume energy proportionally to the rotation.
        """
        amount = abs(angle) * 0.002  # 0.002% per degree
        self.consume_energy(amount)
