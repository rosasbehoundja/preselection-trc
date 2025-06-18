### `sensor_launch.py`

**Rôle** :  
Fichier de lancement ROS2 qui démarre **simultanément** les deux nœuds : `sensor_publisher` et `sensor_subscriber`.

**Fonctionnement** :
- Déclare deux objets `Node` dans la launch description :
  - Un pour `sensor_publisher`
  - Un pour `sensor_subscriber`
- Permet un démarrage automatique des deux nœuds à partir d'une seule commande.

**Commande de lancement** :
```bash
ros2 launch sensor_data_evaluation sensor_launch.py
