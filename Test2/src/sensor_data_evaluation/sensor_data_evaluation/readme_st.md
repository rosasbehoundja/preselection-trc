# Interface Web – `streamlit_app.py`

## Description générale

Cette application Streamlit sert de **dashboard en temps réel** pour la visualisation des données issues de capteurs ROS2 (température, humidité, pression).  

Elle lit les données depuis le fichier `latest_sensor_data.json`, les affiche dynamiquement sous forme de **cartes stylisées**, **graphiques évolutifs**, **tableaux d’historique**, et **statistiques**.  

Elle propose également des **interactions via une sidebar** permettant de contrôler l'affichage, les seuils d’alerte, et le rafraîchissement automatique.

---

## Composants de l’interface utilisateur

### 1. **Sidebar (panneau de contrôle)**

- Bouton **Démarrer / Arrêter** l’affichage des nouvelles données.
- **Effacement de l’historique** en un clic.
- **Configuration dynamique des seuils d’alerte** (via sliders) pour chaque capteur :
  - Température (°C)
  - Humidité (%)
  - Pression (hPa)
- Options de **rafraîchissement** :
  - Activation / désactivation de l'auto-refresh
  - Choix de l’intervalle (1s, 2s, 5s, 10s)
  - Bouton “🔄 Actualiser maintenant”
    
- **Instructions rapides** pour l’utilisateur.
- **État du système** affiché en temps réel :
  - Statut d’affichage
  - Nombre de mesures reçues
  - Compteur de refreshs

---

### 2. **Affichage en temps réel (cartes métriques)**

Trois cartes stylisées, avec couleurs et animations selon l’état :
- 🌡️ **Température**
- 💧 **Humidité**
- 🌪️ **Pression**

Chaque carte affiche :
- Valeur actuelle
- Unité de mesure
- Icône
- État de conformité vis-à-vis des seuils d'alerte (`✅` ou `⚠️`)

> 💡 En cas de dépassement des seuils, la carte clignote et un message d'erreur s’affiche.

---

### 3. **Alertes**

Deux types d’alertes sont affichées :
- **Alerte Dashboard** : basée sur les seuils configurés dans la sidebar.
- **Alerte ROS2** : basée sur les seuils du `sensor_subscriber.py` (fichier JSON).

---

### 4. **Dernière mise à jour**

Un indicateur donne la **date et l’heure** de la dernière mesure reçue et affichée.

---

### 5. **Onglets dynamiques**

#### 📈 **Graphiques (Plotly)**
- 3 graphiques synchronisés (température, humidité, pression)
- Affichage en ligne + points
- Données historisées jusqu’à 100 mesures
- Graphiques interactifs : zoom, survol, export

#### 📋 **Historique**
- Tableau des **20 dernières mesures**
- Affichage formaté : arrondi à 2 décimales, horodatage simplifié
- Présence d’indicateurs de validation (`temp_ok`, `hum_ok`, `pres_ok`)

#### 📊 **Statistiques**
- Moyenne, min et max pour chaque capteur
- Affichage clair dans 3 colonnes
- Mise à jour automatique à chaque ajout de donnée

---

## Fonctionnalités techniques avancées

- **Auto-refresh intelligent** (via `streamlit_autorefresh`) avec synchronisation sur l’état `running`
  
- **Sécurité des données** :
  - Vérification de la structure JSON
  - Lecture sécurisée (try/except)
- **Affichage stylisé** :
  - Utilisation de **CSS custom** pour les composants
  - Animation `pulse` pour signaler un danger
- **Session persistante** :
  - Historique en mémoire dans `st.session_state`
  - Configuration utilisateur conservée au rafraîchissement

---

## Instructions d’utilisation

1. Lancer les scripts ROS 2 :
   ```bash
   ros2 run sensor_data_evaluation sensor_publisher
   ros2 run sensor_data_evaluation sensor_subscriber
   ```

2. Démarrer le dashboard Streamlit :
   ```bash
   streamlit run streamlit_app.py
   ```

3. Ajuster les seuils et visualiser les données en temps réel !
