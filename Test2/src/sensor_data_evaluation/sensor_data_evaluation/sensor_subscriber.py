import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class SensorSubscriber(Node):
    """
    Souscrit à /sensor_data, vérifie si temp, hum, pres sont dans les plages attendues,
    et loggue un message si hors plage ou ok.
    """
    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/sensor_data',
            self.listener_callback,
            10)
        self.subscription  # évite warning
        # Plages acceptables
        self.temp_range = (15.0, 35.0)
        self.hum_range = (30.0, 70.0)
        self.pres_range = (950.0, 1050.0)
        self.get_logger().info('SensorSubscriber initialisé, en attente de données sur /sensor_data.')

    def listener_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) != 3:
            self.get_logger().warning(f'Données inattendues, length={len(data)}')
            return
        temp, hum, pres = data
        ok = True
        if not (self.temp_range[0] <= temp <= self.temp_range[1]):
            self.get_logger().error(f'Température hors plage: {temp:.2f}°C')
            ok = False
        if not (self.hum_range[0] <= hum <= self.hum_range[1]):
            self.get_logger().error(f'Humidité hors plage: {hum:.2f}%')
            ok = False
        if not (self.pres_range[0] <= pres <= self.pres_range[1]):
            self.get_logger().error(f'Pression hors plage: {pres:.2f}hPa')
            ok = False
        if ok:
            self.get_logger().info(f'Données OK: temp={temp:.2f}, hum={hum:.2f}, pres={pres:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('SensorSubscriber arrêt.')
        node.destroy_node()
        rclpy.shutdown()
