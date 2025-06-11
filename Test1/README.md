# Robot Simulation

This project simulates various types of robots, focusing on a robust object-oriented design with a base abstract robot class and two concrete implementations: a wheeled mobile robot and an articulated robotic arm.

## Project Structure

```
Test 1/
├── __init__.py
├── robot.py         # Abstract base class for all robots
├── wheeledRobot.py  # Wheeled mobile robot implementation
└── roboticArm.py    # Articulated robotic arm implementation
```

### Documentation
- [`docs/robot.md`](docs/robot.md) : Documentation for the `Robot` class
- [`docs/wheeledRobot.md`](docs/wheeledRobot.md) : Documentation for the `WheeledRobot` class
- [`docs/roboticArm.md`](docs/roboticArm.md) : Documentation for the `RoboticArm` class

## UML Class Diagram

The following diagram reflects the actual class structure and main methods:

![UML Class Diagram](../media/diagram.png)