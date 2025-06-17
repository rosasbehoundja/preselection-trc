### 1. Compilation
colcon build --packages-select sensor_data_evaluation

### 2. Sourcing
source install/setup.bash

### 3. Exécution séparée
ros2 run sensor_data_evaluation sensor_publisher
ros2 run sensor_data_evaluation sensor_subscriber

### 4. Exécution via launch
ros2 launch sensor_data_evaluation sensor_launch.py
