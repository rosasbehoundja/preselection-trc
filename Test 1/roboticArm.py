from robot import Robot
from typing import Tuple, Union, List
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

    # Properties for the robotic arm
    @property
    def joint_angles(self) -> List[float]:
        """Get the current joint angles (in degrees)."""
        return self._joint_angles.copy()

    @property
    def num_joints(self) -> int:
        return self._num_joints
    
    @joint_angles.setter
    def joint_angles(self, angles: List[float]) -> None:
        if not isinstance(angles, list) or len(angles) != self._num_joints or not all(isinstance(a, (int, float)) for a in angles):
            raise ValueError(f"Angles must be a list of {self._num_joints} numeric values.")
        self._joint_angles = [float(a) for a in angles]
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
        self.position = target_position
        self._logger.info(f"Moved end effector to {self.position} with joint angles {self._joint_angles}")

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
        self.orientation = (self.orientation + angle) % 360
        self._logger.info(f"Rotated joint {joint_index} by {angle} degrees. New angle: {self._joint_angles[joint_index]}")

    def stop(self) -> None:
        """
        Stop all movements of the robotic arm.
        """
        self.is_active = False
        self._joint_angles = [0.0] * self._num_joints  # Reset joint angles
        self._logger.info("Robotic arm stopped.")

    def status(self) -> str:
        """
        Get the current status of the robotic arm.
        """
        return (f"RoboticArm(ID: {self.id}, Name: {self.name}, Position: {self.position}, "
                f"Orientation: {self.orientation}, Joints: {self._joint_angles}, "
                f"Active: {self.is_active}, Generator level: {self.generator_level})")

    def distance_to(self, x: float, y: float) -> float:
        """
        Calculate the distance from the end effector to a target point.
        """
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            raise ValueError("Coordinates must be numeric values.")
        ex, ey = self.position
        return math.hypot(x - ex, y - ey)

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
        Pick up an item with the robotic arm.
        """
        if not self.is_active:
            self._logger.warning(f"Cannot pick {item}: Robotic arm is inactive.")
            return
        print(f"{self.name} is picking up {item} at {self.position}.")
        self._logger.info(f"Picked up {item} at {self.position}.")

    def place(self, item: str, target_position: Tuple[float, float]) -> None:
        """
        Place an item at a target position with the robotic arm.
        """
        if not self.is_active:
            self._logger.warning(f"Cannot place {item}: Robotic arm is inactive.")
            return
        self.move(target_position)
        print(f"{self.name} placed {item} at {target_position}.")
        self._logger.info(f"Placed {item} at {target_position}.")

if __name__ == "__main__":
    # Example usage of the RoboticArm class
    arm = RoboticArm(id=1, name="Arm1", position=(0.0, 0.0), orientation=0.0, energy_source="Electric")
    arm.move((5.0, 5.0))
    arm.rotate(0, 45)
    arm.pick("Box")
    arm.place("Box", (10.0, 10.0))
    print(arm.status())
    arm.reset_arm()
    print(arm.status())
    arm.stop()
    arm.set_joint_angle(1, 90)
    print(arm.joint_angles)