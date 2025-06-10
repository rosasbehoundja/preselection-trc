from .robot import Robot
from typing import Tuple, Union


class RoboticArm(Robot):
    """A class representing a robotic arm, inheriting from the Robot class."""

    def __init__(self, id, name, position, orientation, energy_source):
        super().__init__(id, name, position, orientation, energy_source)

    
    # Personalized methods for RoboticArm
    def move(self, direction, distance):
        return super().move(direction, distance)
    
    def recharge(self):
        return super().recharge()
    
    def rotate(self, angle):
        return super().rotate(angle)
    
    def status(self):
        return super().status()
    
    def stop(self):
        return super().stop()
    
    # Additional methods specific to RoboticArm
    def pick(self, item: str) -> None:
        """
        Pick up an item with the robotic arm.
        
        Args:
            item (str): The name of the item to pick up.
        """
        print(f"{self.name} is picking up {item}.")