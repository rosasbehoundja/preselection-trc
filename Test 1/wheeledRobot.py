from robot import Robot

from typing import Tuple, List, Any
from enum import Enum
import math


class State(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    WAITING_FOR_ARM = "WAITING_FOR_ARM"
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
            energy_source (str): Type of energy source (e.g., "battery").
            wheel_base (float): Distance between the wheels in meters.
            storage_capacity (int): Maximum number of items the robot can store.

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

    @storage_bag.setter
    def storage_bag(self, item: Any) -> None:
        """Add an item to the storage bag."""
        if len(self._storage_bag) >= self._stockage_capacity:
            raise ValueError("Storage bag is full.")
        self._storage_bag.append(item)
        self._logger.info(f"Item added to storage bag. Current count: {len(self._storage_bag)}")

    @obstacle_threshold.setter
    def obstacle_threshold(self, threshold: float) -> None:
        """Set the obstacle detection threshold."""
        if not isinstance(threshold, (int, float)) or threshold <= 0:
            raise ValueError("obstacle_threshold must be a positive number.")
        self._obstacle_threshold = float(threshold)
        self._logger.info(f"Obstacle threshold set to {self._obstacle_threshold:.2f} meters")
    

    # Motors and sensors
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

        angle = self._orientation if direction == "forward" else self._orientation + math.pi

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

    def detect_obstacle(self) -> bool:
        """
        Vérifie si un obstacle est trop proche via les capteurs.
        Retourne True si au moins un capteur lit < threshold.
        """
        dists = self.read_sensors()
        for d in dists:
            if d is not None and d < self._obstacle_threshold:
                self._logger.info(f"Obstacle detected at distance {d:.2f}m")
                return True
        return False

    def avoid_obstacle(self) -> bool:
        """
        Stratégie simple de contournement :
        - Stoppe,
        - Recule un peu,
        - Tourne de 45 degrés,
        - Avance un peu,
        - Puis retourne True pour signaler manœuvre effectuée.
        """
        self._logger.info("Avoiding obstacle...")
        try:
            self.stop()
            self.move("backward", 0.2)
            self.rotate(math.pi / 4)
            self.move("forward", 0.3)
        except RuntimeError as e:
            self._logger.warning(f"Obstacle avoidance failed: {e}")
            return False
        self._logger.info("Obstacle avoidance maneuver done.")
        return True

    def navigate_to(self, target: Tuple[float, float]) -> bool:
        """
        Planifie et suit une trajectoire vers target (x, y).
        Retourne True si arrivé, False si interrompu (batterie faible ou obstacle infranchissable).
        """
        # Vérifie actif & batterie
        if not self.is_active or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
            self._logger.warning("Cannot navigate: inactive or battery low.")
            return False

        path = self._planner.compute_path(self._position, target)
        for waypoint in path:
            # Boucle jusqu'à atteindre le waypoint
            while True:
                # Obstacle ?
                if self.detect_obstacle():
                    ok = self.avoid_obstacle()
                    if not ok:
                        self._logger.warning("Cannot avoid obstacle, abort navigation.")
                        return False
                # Calcul vecteur vers waypoint
                dx = waypoint[0] - self._position[0]
                dy = waypoint[1] - self._position[1]
                dist = math.hypot(dx, dy)
                if dist < 0.05:
                    break  # waypoint atteint
                # Orientation souhaitée
                desired_angle = math.atan2(dy, dx)
                # Calcul de la différence d'angle dans [-π, π]
                angle_diff = (desired_angle - self._orientation + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_diff) > 0.1:
                    # Tourne par pas
                    step = max(min(angle_diff, 0.3), -0.3)
                    try:
                        self.rotate(step)
                    except RuntimeError:
                        return False
                else:
                    # Avance vers waypoint
                    step_dist = min(dist, 0.2)
                    try:
                        self.move("forward", step_dist)
                    except RuntimeError:
                        return False
                # Vérifier batterie en cours de route
                if not self.is_active or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
                    self._logger.warning("Battery too low mid-navigation.")
                    return False
            # waypoint atteint
        self._logger.info(f"Arrived at target {target}")
        return True

    # ---------- Coordination avec le bras ----------

    def prepare_for_pickup(self, cube_pos: Tuple[float, float]) -> bool:
        """
        Positionne le robot pour que le bras puisse saisir le cube.
        cube_pos est la position absolue dans l'arène.
        """
        # Orientation vers le cube
        dx = cube_pos[0] - self._position[0]
        dy = cube_pos[1] - self._position[1]
        desired_angle = math.atan2(dy, dx)
        angle_diff = (desired_angle - self._orientation + math.pi) % (2 * math.pi) - math.pi
        try:
            self.rotate(angle_diff)
            # Approche jusqu'à distance de prise (ex. 0.3 m)
            dist_to_cube = math.hypot(dx, dy)
            approach_dist = max(dist_to_cube - 0.3, 0.0)
            if approach_dist > 0:
                self.move("forward", approach_dist)
        except RuntimeError:
            self._logger.warning("Failed to position for pickup.")
            return False
        # S'assurer que la plateforme est immobile
        self.stop()
        self._logger.info("Positioned for pickup.")
        return True

    def resume_after_pickup(self):
        """
        Réinitialise l'état pour reprendre navigation.
        """
        self._logger.info("Resuming navigation after pickup.")
        # Aucun code spécifique ; la machine à états reprendra la navigation.

    def add_cube_to_storage(self, cube_info: Any) -> bool:
        """
        Appelé après succès du pick par le bras.
        """
        success = self._storage.add(cube_info)
        if success:
            self._logger.info(f"Cube ajouté au stockage ({self._storage.count()}/{self._storage._capacity}).")
        else:
            self._logger.warning("Stockage plein, cube non ajouté.")
        return success

    def is_storage_full(self) -> bool:
        return self._storage.is_full()

    def return_to_sorting_zone(self, sort_zone_pos: Tuple[float, float]) -> bool:
        """
        Lance la navigation vers la zone de tri.
        """
        self._logger.info("Returning to sorting zone.")
        return self.navigate_to(sort_zone_pos)

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
        amount = abs(angle) * 0.002  # ex. 0.002% per degrees
        self.consume_energy(amount)