import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random

class SensorPublisher(Node):
    """
    Publie aléatoirement température, humidité, pression toutes les 0.5s sur /sensor_data
    sous forme de Float32MultiArray à 3 éléments [temp, hum, pres].
    """
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/sensor_data', 10)
        timer_period = 0.5  # secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('SensorPublisher initialisé, publication toutes les 0.5s.')

    def timer_callback(self):
        # Générer valeurs aléatoires dans les plages :
        temp = random.uniform(15.0, 35.0)      # °C
        hum = random.uniform(30.0, 70.0)       # %
        pres = random.uniform(950.0, 1050.0)   # hPa

        msg = Float32MultiArray()
        msg.data = [float(temp), float(hum), float(pres)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publié: temp={temp:.2f}°C, hum={hum:.2f}%, pres={pres:.2f}hPa')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('SensorPublisher arrêt.')
        node.destroy_node()
        rclpy.shutdown()
