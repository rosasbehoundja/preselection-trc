from .robot import Robot
from typing import Tuple, Union


class WheeledRobot(Robot):
    """A class representing a wheeled robot, inheriting from the Robot class."""
    
    def __init__(self, id, name, position, orientation, energy_source):
        super().__init__(id, name, position, orientation, energy_source)

    # Personalized methods for WheeledRobot
    def move(self, direction, distance):
        return super().move(direction, distance)

    def rotate(self, angle):
        return super().rotate(angle)

    def recharge(self):
        return super().recharge()

    def stop(self):
        return super().stop()

    def status(self):
        return super().status()
    
    def distance_to(self, *args):
        return super().distance_to(*args)
    
    # Additional methods specific to WheeledRobot
    def drive(self, speed: float, duration: float) -> None:
        """
        Drive the wheeled robot at a specified speed for a certain duration.
        
        Args:
            speed (float): The speed at which to drive the robot.
            duration (float): The duration for which to drive the robot.
        """
        print(f"{self.name} is driving at {speed} m/s for {duration} seconds.")
