from robot import Robot
from typing import Tuple, List, Optional
import math


class RoboticArm(Robot):
    """
    A class representing an articulated robotic arm, inheriting from the Robot class.
    The arm consists of multiple joints (shoulder, elbow, wrist, etc.), each with its own angle.
    """

    def __init__(
            self,
            name: str,
            position: Tuple[float, float],
            orientation: float,
            energy_source: str,
            num_joints: int = 6
    ) -> None:
        """
        Initialize the robotic arm with the given parameters.
        """
        super().__init__(name=name, position=position, orientation=orientation, energy_source=energy_source)
        if not isinstance(num_joints, int) or num_joints < 2:
            raise ValueError("A robotic arm must have at least 2 joints.")
        self._joint_angles: List[float] = [0.0] * num_joints  # Angles in degrees
        self._num_joints = num_joints
        self._end_effector_position: Tuple[float, float] = position
        self._holding_item: Optional[str] = None

    # Properties for the robotic arm
    @property
    def joint_angles(self) -> List[float]:
        """Get the current joint angles (in degrees)."""
        return self._joint_angles.copy()

    @property
    def num_joints(self) -> int:
        return self._num_joints
    
    @property
    def end_effector_position(self) -> Tuple[float, float]:
        return self._end_effector_position

    @property
    def holding_item(self) -> Optional[str]:
        return self._holding_item
    
    @joint_angles.setter
    def joint_angles(self, angles: List[float]) -> None:
        if not isinstance(angles, list) or len(angles) != self._num_joints or not all(isinstance(a, (int, float)) for a in angles):
            raise ValueError(f"Angles must be a list of {self._num_joints} numeric values.")
        self._joint_angles = [float(a % 360) for a in angles]
        self._logger.info(f"Joint angles updated: {self._joint_angles}")

    # Personalized methods for RoboticArm
    def move(self, target_position: Tuple[float, float]) -> None:
        """
        Move the end effector to a target position (2D for simplicity).
        """
        if not self.is_active:
            self._logger.warning("Cannot move: Robotic arm is inactive.")
            return
        if not (isinstance(target_position, tuple) and len(target_position) == 2):
            raise ValueError("Target position must be a tuple (x, y).")
        
        angle = math.degrees(math.atan2(target_position[1] - self.position[1], target_position[0] - self.position[0]))
        self._joint_angles = [angle / self._num_joints] * self._num_joints
        self._end_effector_position = target_position
        self.consume_energy(1.0)
        self._logger.info(f"Moved end effector to {target_position} with joint angles {self._joint_angles}")

    def rotate(self, joint_index: int, angle: float) -> None:
        """
        Rotate a specific joint by a given angle (in degrees).
        """
        if not self.is_active:
            self._logger.warning("Cannot rotate: Robotic arm is inactive.")
            return
        if not (0 <= joint_index < self._num_joints):
            raise IndexError("Invalid joint index.")
        
        self._joint_angles[joint_index] = (self._joint_angles[joint_index] + angle) % 360
        self.orientation = (math.degrees(self.orientation) + angle) % 360
        self.consume_energy(0.1)
        self._logger.info(f"Rotated joint {joint_index} by {angle} radians. New angle: {self._joint_angles[joint_index]}")

    def stop(self) -> None:
        """
        Stop all movements of the robotic arm.
        """
        self.is_active = False
        self._logger.info("Robotic arm stopped.")

    def status(self) -> str:
        return (f"RoboticArm(ID: {self.id}, Name: {self.name}, Position: {self.position}, "
                f"End-Effector: {self._end_effector_position}, Orientation: {self.orientation:.2f} rad, "
                f"Joints: {self._joint_angles}, Holding: {self._holding_item}, "
                f"Active: {self.is_active}, Generator level: {self.generator_level})")

    def set_joint_angle(self, joint_index: int, angle: float) -> None:
        """
        Set a specific joint to a given angle (in degrees).
        """
        if not (0 <= joint_index < self._num_joints):
            raise IndexError("Invalid joint index.")
        self._joint_angles[joint_index] = angle % 360
        self._logger.info(f"Set joint {joint_index} to {angle} degrees.")

    def reset_arm(self) -> None:
        """
        Reset all joint angles to zero and move end effector to base position.
        """
        self._joint_angles = [0.0] * self._num_joints
        self._logger.info("Robotic arm reset to home position.")

    def pick(self, item: str) -> None:
        """
        Pick up an item with the robotic arm's end effector."""
        if not self.is_active:
            self._logger.warning(f"Cannot pick {item}: Robotic arm is inactive.")
            return
        if self._holding_item is not None:
            self._logger.warning(f"Already holding {self._holding_item}, cannot pick {item}.")
            return
        self._holding_item = item
        self.consume_energy(0.5)
        self._logger.info(f"Picked up {item} at {self._end_effector_position}.")

    def place(self, item: str, target_position: Tuple[float, float]) -> None:
        """
        Place the currently held item at a target position or Storage Bag.
        """
        if not self.is_active:
            self._logger.warning(f"Cannot place {item}: Robotic arm is inactive.")
            return
        if self._holding_item != item:
            self._logger.warning(f"Cannot place {item}: Not currently holding it.")
            return
        self.move(target_position)
        self._holding_item = None
        self.consume_energy(0.5)
        self._logger.info(f"Placed {item} at {target_position}.")

    def reset_arm(self) -> None:
        self._joint_angles = [0.0] * self._num_joints
        self._end_effector_position = self.position
        self._holding_item = None
        self._logger.info("Robotic arm reset to home position.")
