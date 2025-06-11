# Robot Simulation Project

This project simulates various types of robots, focusing on a robust object-oriented design with a base abstract robot class and two concrete implementations: a wheeled mobile robot and an articulated robotic arm.

## Project Structure

```
Test 1/
├── __init__.py
├── robot.py         # Abstract base class for all robots
├── wheeledRobot.py  # Wheeled mobile robot implementation
└── roboticArm.py    # Articulated robotic arm implementation
```

## Class Overview

### `Robot` (robot.py)
- **Type:** Abstract Base Class
- **Attributes:**
  - `id`: Unique identifier (UUID)
  - `name`: Robot name
  - `position`: (x, y) coordinates (float, float)
  - `orientation`: Orientation in radians
  - `energy_source`: "solar", "fossil_fuel", or "electric"
  - `generator_level`: Power level (int)
  - `is_active`: Whether the robot is active (bool)
  - `sensors`: List of attached sensors
- **Key Methods:**
  - `move(direction, distance)`: Abstract, move robot in a direction by a distance
  - `rotate(angle)`: Abstract, rotate robot by an angle (radians)
  - `stop()`: Abstract, stop robot movement
  - `status()`: Abstract, get robot status as string
  - `start()`: Activate the robot
  - `consume_energy(amount)`: Consume energy from the generator
  - `add_sensor(sensor)`, `remove_sensor(sensor)`: Manage sensors
  - `distance_to(other)`: Compute distance to another point

### `WheeledRobot` (wheeledRobot.py)
- **Inherits:** `Robot`
- **Additional Attributes:**
  - `wheel_base`: Distance between wheels (float)
  - `storage_capacity`: Max items in storage (int)
  - `storage_bag`: List of stored items
  - `state`: Current state (Enum: IDLE, NAVIGATING, etc.)
  - `obstacle_threshold`: Minimum distance to detect obstacles
  - `v_lin_cmd`, `v_ang_cmd`: Linear and angular speed commands
- **Key Methods:**
  - `set_motor_speed(left, right)`: Set wheel speeds
  - `add_to_storage(item)`: Add item to storage
  - `move(direction, distance)`: Move forward/backward
  - `rotate(angle)`: Rotate robot
  - `stop()`: Stop movement
  - `detect_obstacle(obstacle_position)`: Detect nearby obstacles
  - `avoid_obstacle()`: Perform obstacle avoidance
  - `is_storage_full()`: Check if storage is full
  - `status()`: Get robot status

### `RoboticArm` (roboticArm.py)
- **Inherits:** `Robot`
- **Additional Attributes:**
  - `num_joints`: Number of joints (int)
  - `joint_angles`: List of joint angles (degrees)
  - `end_effector_position`: (x, y) position of the end effector
  - `holding_item`: Currently held item (optional)
- **Key Methods:**
  - `move(target_position)`: Move end effector to a position
  - `rotate(joint_index, angle)`: Rotate a specific joint
  - `stop()`: Stop all movements
  - `status()`: Get arm status
  - `set_joint_angle(joint_index, angle)`: Set a joint to a specific angle
  - `reset_arm()`: Reset all joints and end effector
  - `pick(item)`: Pick up an item
  - `place(item, target_position)`: Place held item at a position

## UML Class Diagram

The following diagram reflects the actual class structure and main methods:

![UML Class Diagram](../media/diagram.png)