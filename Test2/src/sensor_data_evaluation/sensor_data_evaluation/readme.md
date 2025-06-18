## Documentation des fichiers principaux

---

### 1- `sensor_publisher.py`

**Rôle** :  
Node ROS2 qui publie des mesures aléatoires de température (15–35 °C), humidité (30–70 %) et pression (950–1050 hPa) toutes les 0,5 s sur le topic `/sensor_data`.

**Fonctionnement** :
- Création d’un publisher ROS2 `Float32MultiArray` (3 floats).
- Timer ROS2 (`create_timer`) à 0,5 s pour générer de nouvelles valeurs aléatoires et publier.
- Logs d’info affichés à chaque publication.

**Points d’extension** :
- Paramétrage de la fréquence de publication et des plages via les paramètres ROS2.
- Passage à des messages ROS2 personnalisés si nécessaire.

**Usage** :
```bash
ros2 run sensor_data_evaluation sensor_publisher
```
ou
```bash
python sensor_publisher.py
```

---

### 2- `sensor_subscriber.py`

**Rôle :**
Node ROS2 qui souscrit au topic `/sensor_data`, vérifie les plages des mesures, loggue les alertes et écrit la dernière mesure dans un fichier JSON (`latest_sensor_data.json`) utilisé par l’interface Streamlit.

**Fonctionnement :**

- Souscription ROS2 à `Float32MultiArray`.

- Dans le callback :

  - Vérification de la taille (3 éléments).
  
  - Vérification des plages :

    - Température : 15–35 °C
    
    - Humidité : 30–70 %
    
    - Pression : 950–1050 hPa

  - Log niveau `error` si valeur hors plage, sinon log `info`.

  - Création d’un dictionnaire contenant :

    - `temperature`, `humidity`, `pressure`
    
    - `timestamp` (format ISO)
    
    - `temp_ok`, `hum_ok`, `pres_ok` (booléens)

  - Écriture atomique recommandée : écrire dans un fichier temporaire puis utiliser `os.replace` pour éviter les lectures incomplètes.

**Points d’extension :**

- Paramètres ROS2 pour le chemin du fichier JSON et les plages dynamiques.

- Ajout d’un enregistrement d’historique (CSV, SQLite) en plus du JSON.

**Usage :**
```bash
ros2 run sensor_data_evaluation sensor_subscriber
```
ou
```bash
python sensor_subscriber.py
```
