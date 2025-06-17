### 1. Compilation
```bash
colcon build --packages-select sensor_data_evaluation
```

### 2. Sourcing
```bash
source install/setup.bash
```

### 3. Exécution séparée
**Publisher:** Dans Terminal 1
```bash
ros2 run sensor_data_evaluation sensor_publisher
```
**Subscriber:** Dans Terminal 2
```bash
ros2 run sensor_data_evaluation sensor_subscriber
```

### 4. Exécution via launch
```bash
ros2 launch sensor_data_evaluation sensor_launch.py
```
