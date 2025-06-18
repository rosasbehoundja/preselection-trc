import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
from datetime import datetime

class SensorSubscriber(Node):
    """
    Souscrit à /sensor_data, vérifie si temp, hum, pres sont dans les plages attendues,
    et sauvegarde les données dans un fichier JSON pour Streamlit.
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
        
        # Vérifier les plages
        temp_ok = self.temp_range[0] <= temp <= self.temp_range[1]
        hum_ok = self.hum_range[0] <= hum <= self.hum_range[1]
        pres_ok = self.pres_range[0] <= pres <= self.pres_range[1]
        
        # Logging des erreurs
        if not temp_ok:
            self.get_logger().error(f'Température hors plage: {temp:.2f}°C')
        if not hum_ok:
            self.get_logger().error(f'Humidité hors plage: {hum:.2f}%')
        if not pres_ok:
            self.get_logger().error(f'Pression hors plage: {pres:.2f}hPa')
        
        if temp_ok and hum_ok and pres_ok:
            self.get_logger().info(f'Données OK: temp={temp:.2f}, hum={hum:.2f}, pres={pres:.2f}')
        
        # Préparer les données pour Streamlit
        sensor_data = {
            'temperature': float(temp),
            'humidity': float(hum),
            'pressure': float(pres),
            'timestamp': datetime.now().isoformat(),
            'temp_ok': temp_ok,
            'hum_ok': hum_ok,
            'pres_ok': pres_ok
        }
        
        # Sauvegarder dans le fichier JSON pour Streamlit
        try:
            with open('latest_sensor_data.json', 'w') as f:
                json.dump(sensor_data, f, indent=2)
        except Exception as e:
            self.get_logger().warning(f'Erreur sauvegarde: {e}')

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

if __name__ == '__main__':
    main()
