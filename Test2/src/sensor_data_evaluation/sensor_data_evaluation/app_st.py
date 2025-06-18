import streamlit as st
from streamlit_autorefresh import st_autorefresh
import time
import json
import pandas as pd
from datetime import datetime
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import asyncio
import threading

# Configuration de la page
st.set_page_config(
    page_title="Dashboard Capteurs ROS2",
    page_icon="🌡️",
    layout="wide",
    initial_sidebar_state="expanded"
)

# CSS personnalisé pour le style
st.markdown("""
<style>
    .metric-card {
        background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
        padding: 1rem;
        border-radius: 10px;
        color: white;
        text-align: center;
        margin: 0.5rem 0;
        box-shadow: 0 4px 6px rgba(0,0,0,0.1);
    }
    
    .metric-card-danger {
        background: linear-gradient(135deg, #ff6b6b 0%, #ee5a24 100%);
        animation: pulse 1s infinite;
    }
    
    .metric-card-success {
        background: linear-gradient(135deg, #00b894 0%, #00cec9 100%);
    }
    
    @keyframes pulse {
        0% { opacity: 1; }
        50% { opacity: 0.7; }
        100% { opacity: 1; }
    }
    
    .main-title {
        text-align: center;
        color: #2c3e50;
        font-size: 2.5rem;
        margin-bottom: 2rem;
        text-shadow: 2px 2px 4px rgba(0,0,0,0.1);
    }
    
    .stTabs [data-baseweb="tab"] {
        font-size: 1.2rem;
        font-family: 'Arial', sans-serif;
        font-weight: bold;
        padding: 1rem 2rem;
        margin: 0.5rem;
        border-radius: 10px;
        background-color: #f2f2f2;
        color: #333;
        border: 2px solid #ccc;
        box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        transition: all 0.3s ease-in-out;
        text-align: center;
    }

    .stTabs [aria-selected="true"] {
        background-color: #4CAF50 !important;
        color: white !important;
        border-color: #4CAF50;
    }
</style>
""", unsafe_allow_html=True)

# Initialisation des données de session
def initialize_session_state():
    if 'sensor_data_history' not in st.session_state:
        st.session_state.sensor_data_history = []
    if 'running' not in st.session_state:
        st.session_state.running = True
    if 'last_update_time' not in st.session_state:
        st.session_state.last_update_time = 0
    if 'refresh_counter' not in st.session_state:
        st.session_state.refresh_counter = 0

# Version améliorée du cache pour éviter les problèmes d'async
def load_sensor_data():
    try:
        with open('latest_sensor_data.json', 'r') as f:
            data = json.load(f)
            data['timestamp'] = datetime.fromisoformat(data['timestamp'])
            return data
    except FileNotFoundError:
        return None
    except Exception as e:
        st.error(f"Erreur lecture fichier: {e}")
        return None

# Fonction pour afficher une métrique avec indicateur de statut
def display_metric_card(title, value, unit, emoji, is_normal, threshold_info=""):
    card_class = "metric-card-success" if is_normal else "metric-card-danger"
    status_emoji = "✅" if is_normal else "⚠️"
    
    st.markdown(f"""
    <div class="metric-card {card_class}">
        <h3>{emoji} {title}</h3>
        <h1>{value:.2f} {unit}</h1>
        <p>{status_emoji} {threshold_info}</p>
    </div>
    """, unsafe_allow_html=True)

# Fonction pour créer les graphiques
def create_charts(data_history):
    if len(data_history) < 2:
        return None
    
    try:
        df = pd.DataFrame(data_history)
        
        # Créer un graphique avec 3 sous-graphiques
        fig = make_subplots(
            rows=3, cols=1,
            subplot_titles=('🌡️ Température (°C)', '💧 Humidité (%)', '🌪️ Pression (hPa)'),
            vertical_spacing=0.15
        )
        
        # Température
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['temperature'], 
                      mode='lines+markers', name='Température',
                      line=dict(color='#e74c3c', width=3)),
            row=1, col=1
        )
        
        # Humidité
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['humidity'], 
                      mode='lines+markers', name='Humidité',
                      line=dict(color='#3498db', width=3)),
            row=2, col=1
        )
        
        # Pression
        fig.add_trace(
            go.Scatter(x=df['timestamp'], y=df['pressure'], 
                      mode='lines+markers', name='Pression',
                      line=dict(color='#9b59b6', width=3)),
            row=3, col=1
        )
        
        fig.update_layout(height=600, showlegend=False, 
                         title_text="Évolution des capteurs en temps réel")
        
        return fig
    except Exception as e:
        st.error(f"Erreur lors de la création du graphique: {e}")
        return None

# Fonction pour calculer les statistiques
def calculate_stats(data_history):
    if not data_history:
        return None
    
    try:
        df = pd.DataFrame(data_history)
        
        stats = {
            'temperature': {
                'mean': df['temperature'].mean(),
                'max': df['temperature'].max(),
                'min': df['temperature'].min()
            },
            'humidity': {
                'mean': df['humidity'].mean(),
                'max': df['humidity'].max(),
                'min': df['humidity'].min()
            },
            'pressure': {
                'mean': df['pressure'].mean(),
                'max': df['pressure'].max(),
                'min': df['pressure'].min()
            }
        }
        
        return stats
    except Exception as e:
        st.error(f"Erreur lors du calcul des statistiques: {e}")
        return None

# Fonction d'auto-refresh personnalisée plus stable
def auto_refresh_custom(interval_seconds=1):
    """Auto-refresh personnalisé sans utiliser streamlit_autorefresh"""
    current_time = time.time()
    
    if 'last_refresh_time' not in st.session_state:
        st.session_state.last_refresh_time = current_time
    
    # Si assez de temps s'est écoulé, déclencher un refresh
    if current_time - st.session_state.last_refresh_time >= interval_seconds:
        st.session_state.last_refresh_time = current_time
        st.session_state.refresh_counter += 1
        # Utiliser st.rerun() au lieu de st.experimental_rerun()
        if st.session_state.refresh_counter % 2 == 0:  # Réduire la fréquence
            st.rerun()

# Interface principale
def main():
    try:
        initialize_session_state()
        
        # Titre principal
        st.markdown('<h1 class="main-title">🌡️ Dashboard Capteurs ROS2</h1>', unsafe_allow_html=True)
        
        # Sidebar pour les contrôles
        with st.sidebar:
            st.header("⚙️ Contrôles")
            
            # Bouton start/stop pour l'affichage seulement
            if st.button("🟢 Démarrer affichage" if not st.session_state.running else "🔴 Arrêter affichage"):
                st.session_state.running = not st.session_state.running
            
            # Option pour effacer l'historique
            if st.button("🗑️ Effacer historique"):
                st.session_state.sensor_data_history = []
                st.success("Historique effacé!")
            
            # Configuration des seuils d'affichage
            st.subheader("🎯 Seuils d'alerte (affichage)")
            temp_min = st.slider("Température min (°C)", 0, 20, 15)
            temp_max = st.slider("Température max (°C)", 25, 50, 35)
            hum_min = st.slider("Humidité min (%)", 0, 40, 30)
            hum_max = st.slider("Humidité max (%)", 50, 100, 70)
            pres_min = st.slider("Pression min (hPa)", 900, 1000, 950)
            pres_max = st.slider("Pression max (hPa)", 1000, 1100, 1050)
            
            # Options de refresh améliorées
            st.subheader("🔄 Actualisation")
            auto_refresh = st.checkbox("🔄 Actualisation automatique", key="auto_refresh", value=True)
            refresh_interval = st.selectbox("Intervalle de refresh", 
                                          options=[1, 2, 5, 10], 
                                          index=1, 
                                          format_func=lambda x: f"{x} seconde{'s' if x > 1 else ''}")
            
            if st.button("🔄 Actualiser maintenant"):
                st.rerun()
            
            st.markdown("---")
            st.markdown("""
            **Instructions:**
            1. Lancez `sensor_publisher.py` dans un terminal
            2. Lancez `sensor_subscriber.py` dans un autre terminal
            3. Cette page affiche les données automatiquement
            """)
            
            # Informations sur l'état
            st.subheader("📊 État du système")
            st.write(f"**Affichage:** {'🟢 Actif' if st.session_state.running else '🔴 Inactif'}")
            st.write(f"**Données collectées:** {len(st.session_state.sensor_data_history)}")
            st.write(f"**Refresh counter:** {st.session_state.refresh_counter}")
        
        # Auto-refresh si activé
        if st.session_state.auto_refresh and st.session_state.running:
            st_autorefresh(interval=refresh_interval * 1000, key="autorefresh")    # En millisecondes
     
           
        # Charger les données depuis le fichier
        current_data = load_sensor_data()
        
        if current_data is None:
            st.error("🔴 Aucune donnée disponible. Vérifiez que sensor_publisher.py et sensor_subscriber.py sont lancés.")
            st.info("📋 Lancez d'abord les scripts ROS2 pour voir les données s'afficher ici.")
            return
        
        # Ajouter les données actuelles à l'historique
        if st.session_state.running:
            # Éviter les doublons en vérifiant le timestamp
            if not st.session_state.sensor_data_history or \
               st.session_state.sensor_data_history[-1]['timestamp'] != current_data['timestamp']:
                st.session_state.sensor_data_history.append(current_data)
                # Garder seulement les 100 dernières valeurs
                if len(st.session_state.sensor_data_history) > 100:
                    st.session_state.sensor_data_history.pop(0)
        
        # Vérifier les plages (pour l'affichage dashboard)
        temp_ok = temp_min <= current_data['temperature'] <= temp_max
        hum_ok = hum_min <= current_data['humidity'] <= hum_max
        pres_ok = pres_min <= current_data['pressure'] <= pres_max
        
        # AFFICHAGE TEMPS RÉEL EN HAUT
        st.subheader("📊 Données en temps réel")
        
        # Affichage en colonnes
        col1, col2, col3 = st.columns(3)
        
        with col1:
            temp_emoji = "🔥" if current_data['temperature'] > 35 else "🌡️"
            temp_info = f"Normal ({temp_min}-{temp_max}°C)" if temp_ok else "⚠️ HORS PLAGE"
            display_metric_card("Température", current_data['temperature'], "°C", 
                              temp_emoji, temp_ok, temp_info)
        
        with col2:
            hum_info = f"Normal ({hum_min}-{hum_max}%)" if hum_ok else "⚠️ HORS PLAGE"
            display_metric_card("Humidité", current_data['humidity'], "%", 
                              "💧", hum_ok, hum_info)
        
        with col3:
            pres_info = f"Normal ({pres_min}-{pres_max}hPa)" if pres_ok else "⚠️ HORS PLAGE"
            display_metric_card("Pression", current_data['pressure'], "hPa", 
                              "🌪️", pres_ok, pres_info)
        
        # Alerte générale (basée sur les seuils dashboard)
        if not (temp_ok and hum_ok and pres_ok):
            st.error("🚨 ATTENTION: Certaines valeurs sont hors des plages d'alerte dashboard!")
        else:
            st.success("✅ Toutes les valeurs sont dans les plages dashboard normales")
        
        # Afficher aussi les alertes ROS2 (du subscriber)
        ros_alerts = []
        if not current_data.get('temp_ok', True):
            ros_alerts.append("Température hors plage ROS2")
        if not current_data.get('hum_ok', True):
            ros_alerts.append("Humidité hors plage ROS2")
        if not current_data.get('pres_ok', True):
            ros_alerts.append("Pression hors plage ROS2")
        
        if ros_alerts:
            st.warning(f"⚠️ Alertes ROS2: {', '.join(ros_alerts)}")
        
        # Dernière mise à jour
        st.info(f"🕐 Dernière mise à jour: {current_data['timestamp'].strftime('%H:%M:%S')}")
        
        # Onglets pour les données historiques
        tab1, tab2, tab3 = st.tabs(["📈 Graphiques", "📋 Historique", "📊 Statistiques"])
        
        with tab1:
            if len(st.session_state.sensor_data_history) > 1:
                st.subheader("📈 Évolution en temps réel")
                chart = create_charts(st.session_state.sensor_data_history)
                if chart:
                    st.plotly_chart(chart, use_container_width=True)
            else:
                st.info("Collecte de données en cours... Les graphiques apparaîtront bientôt.")
        
        with tab2:
            st.subheader("📋 Historique des 20 dernières mesures")
            if st.session_state.sensor_data_history:
                recent_data = st.session_state.sensor_data_history[-20:]
                df_display = pd.DataFrame(recent_data)
                df_display['timestamp'] = df_display['timestamp'].dt.strftime('%H:%M:%S')
                df_display = df_display.round(2)
                # Réorganiser les colonnes
                columns_order = ['timestamp', 'temperature', 'humidity', 'pressure', 'temp_ok', 'hum_ok', 'pres_ok']
                df_display = df_display[columns_order]
                st.dataframe(df_display, use_container_width=True)
            else:
                st.info("Aucune donnée historique disponible")
        
        with tab3:
            st.subheader("📊 Statistiques de la session")
            stats = calculate_stats(st.session_state.sensor_data_history)
            if stats:
                col1, col2, col3 = st.columns(3)
                
                with col1:
                    st.metric("🌡️ Temp Moyenne", f"{stats['temperature']['mean']:.1f}°C")
                    st.metric("🌡️ Temp Max", f"{stats['temperature']['max']:.1f}°C")
                    st.metric("🌡️ Temp Min", f"{stats['temperature']['min']:.1f}°C")
                
                with col2:
                    st.metric("💧 Hum Moyenne", f"{stats['humidity']['mean']:.1f}%")
                    st.metric("💧 Hum Max", f"{stats['humidity']['max']:.1f}%")
                    st.metric("💧 Hum Min", f"{stats['humidity']['min']:.1f}%")
                
                with col3:
                    st.metric("🌪️ Pres Moyenne", f"{stats['pressure']['mean']:.1f}hPa")
                    st.metric("🌪️ Pres Max", f"{stats['pressure']['max']:.1f}hPa")
                    st.metric("🌪️ Pres Min", f"{stats['pressure']['min']:.1f}hPa")
            else:
                st.info("Aucune statistique disponible")
    
    except Exception as e:
        st.error(f"Erreur dans l'application: {e}")
        st.exception(e)

if __name__ == "__main__":
    main()