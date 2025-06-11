import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Test1.src import WheeledRobot
import math


print("--- WheeledRobot Scenario: Navigation and Collection ---")

# 1. Initialize the WheeledRobot
explorer = WheeledRobot(
    name="ExplorerBot",
    position=(0.0, 0.0),
    orientation=0.0,
    energy_source="electric",
    wheel_base=0.4,
    storage_capacity=5
)
explorer.generator_level = 100
print(explorer.status())

# 2. Start the robot
explorer.start()
print(f"Robot '{explorer.name}' is now active: {explorer.is_active}")

# 3. Move to a target location
target_item_location = (5.0, 0.0)
distance_to_item = explorer.distance_to(target_item_location)
print(f"Distance to item at {target_item_location}: {distance_to_item:.2f}m")

explorer.move("forward", distance_to_item)
print(explorer.status())

# 4. Simulate finding an item and add it to storage
item_found = {"id": "rock_A", "value": 10}
if not explorer.is_storage_full():
    explorer.add_to_storage(item_found)
    print(f"Picked up item: {item_found['id']}. Storage: {len(explorer.storage_bag)}/{explorer.storage_capacity}")
else:
    print("Could not pick up item, storage is full.")
print(explorer.status())

# 5. Encounter an obstacle and avoid it
obstacle_coords = (explorer.position[0] + 0.2, explorer.position[1])
explorer.obstacle_threshold = 0.3
if explorer.detect_obstacle(obstacle_coords):
    print(f"Obstacle detected at {obstacle_coords}. Attempting to avoid.")
    explorer.avoid_obstacle()
else:
    print("Path clear.")
print(explorer.status())

# 6. Rotate and move again
explorer.rotate(math.pi)
explorer.move("forward", 1.0)
print(explorer.status())

# 7. Stop the robot
explorer.stop()
print(f"Robot '{explorer.name}' stopped. Active: {explorer.is_active}")
print("--- Scenario End ---")