import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Test1.src import RoboticArm

print("--- RoboticArm Scenario: Pick and Place ---")

# 1. Initialize the RoboticArm
factory_arm = RoboticArm(
    name="AssemblerAlpha",
    position=(0.0, 0.0),
    orientation=0.0,
    energy_source="electric",
    num_joints=5
)
factory_arm.generator_level = 100
print(factory_arm.status())

# 2. Start the arm
factory_arm.start()
print(f"Arm '{factory_arm.name}' is now active: {factory_arm.is_active}")

# 3. Move to the location of an item to pick up
item_pickup_location = (0.7, 0.3)
print(f"Moving to pick up item at {item_pickup_location}...")
factory_arm.move(item_pickup_location)
print(factory_arm.status())

# 4. Pick up the item
item_name = "Gearbox_Part_001"
factory_arm.pick(item_name)
print(f"Picked up '{item_name}'. Holding: {factory_arm.holding_item}")
print(factory_arm.status())

# 5. Move to the placement location
item_placement_location = (0.2, 0.8)
print(f"Moving to place item at {item_placement_location}...")
factory_arm.move(item_placement_location)
print(factory_arm.status())

# 6. Place the item
factory_arm.place(item_name, item_placement_location)
print(f"Placed '{item_name}'. Holding: {factory_arm.holding_item}")
print(factory_arm.status())

# 7. Reset the arm
print("Resetting arm...")
factory_arm.reset_arm()
print(factory_arm.status())

# 8. Stop the arm
factory_arm.stop()
print(f"Arm '{factory_arm.name}' stopped. Active: {factory_arm.is_active}")