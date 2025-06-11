from robot import Robot  # On suppose que Robot définit bien battery_level, _check_active(), consume_energy(), status_base(), etc.

from abc import ABC, abstractmethod
from typing import Tuple, List, Any, Optional
import math
import logging
import time


class Sensor(ABC):
    """
    Interface de base pour capteurs de distance/obstacle.
    Chaque capteur doit implémenter read() retournant une distance en mètres.
    """
    @abstractmethod
    def read(self) -> float:
        """
        Lire la mesure actuelle (distance). 
        Retourne un float (distance en m); on peut renvoyer float('inf') si pas d'obstacle.
        """
        pass


class UltrasonicSensor(Sensor):
    """
    Stub d'un capteur ultrason. 
    En simulation, on peut injecter une fonction de génération de données.
    """
    def __init__(self, get_distance_func: Optional[Any] = None):
        # get_distance_func: callable renvoyant float en simulation ou None
        self._get_distance = get_distance_func

    def read(self) -> float:
        if self._get_distance:
            try:
                dist = self._get_distance()
                # On s'assure que c'est un float ou None
                if dist is None:
                    return float('inf')
                return float(dist)
            except Exception:
                return float('inf')
        # Par défaut, absence d'obstacle simulée
        return float('inf')


class Storage:
    """
    Gère le bac à cubes embarqué.
    """
    def __init__(self, capacity: int):
        if capacity <= 0:
            raise ValueError("Storage capacity must be positive.")
        self._capacity = capacity
        self._content: List[Any] = []

    def add(self, cube_info: Any) -> bool:
        """Ajoute cube_info si pas plein. Retourne True si réussi, False sinon."""
        if self.is_full():
            return False
        self._content.append(cube_info)
        return True

    def clear(self):
        """Vide le bac (après tri)."""
        self._content.clear()

    def is_full(self) -> bool:
        return len(self._content) >= self._capacity

    def count(self) -> int:
        return len(self._content)

    def get_all(self) -> List[Any]:
        """Retourne la liste des cubes stockés."""
        return list(self._content)


class ArmInterface(ABC):
    """
    Interface du bras robotisé embarqué.
    Le RobotMobile appelle ces méthodes pour coordonner la prise.
    """
    @abstractmethod
    def detect_cube(self) -> Optional[Tuple[float, float]]:
        """
        Tente de localiser un cube dans le repère du robot mobile.
        Retourne la position relative (x, y) du cube si détecté, sinon None.
        """
        pass

    @abstractmethod
    def pick(self, cube_position: Tuple[float, float]) -> bool:
        """
        Tente de saisir le cube à la position donnée (relative).
        Retourne True si succès, False sinon.
        """
        pass

    @abstractmethod
    def place_into_storage(self) -> bool:
        """
        Commande le bras pour déposer le cube saisi dans le bac du mobile.
        Retourne True si succès.
        """
        pass


class SimplePlanner:
    """
    Stub de planificateur de trajectoire sur grille/simplifié.
    Ici, on considère que le path est une liste de waypoints (x,y).
    En simulation simple, on retourne directement [target].
    """
    def compute_path(self, start: Tuple[float, float], goal: Tuple[float, float]) -> List[Tuple[float, float]]:
        # Implémentation simplifiée : aller directement au goal
        return [goal]


class State:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    AVOIDING = "AVOIDING"
    WAITING_FOR_ARM = "WAITING_FOR_ARM"
    UPDATING_STORAGE = "UPDATING_STORAGE"
    RETURNING = "RETURNING"
    CHARGING = "CHARGING"
    SHUTDOWN = "SHUTDOWN"


class RobotMobile(Robot):
    """
    Implémentation d'un robot mobile :
    - Navigation / déplacement
    - Détection et évitement d'obstacles
    - Stockage de cubes
    - Coordination avec un bras robotisé (ArmInterface)
    - Gestion d'énergie et machine à états
    """
    def __init__(
        self,
        id: Any,
        name: str,
        position: Tuple[float, float],
        orientation: float,
        energy_source: str,
        wheel_base: float,
        storage_capacity: int,
        arm: ArmInterface,
        sensors: Optional[List[Sensor]] = None,
    ) -> None:
        # Appel de la classe de base ; on suppose que Robot gère battery_level, is_active, etc.
        super().__init__(id=id, name=name, position=position, orientation=orientation, energy_source=energy_source)

        # Paramètres châssis
        if wheel_base <= 0:
            raise ValueError("wheel_base must be positive.")
        self._wheel_base = float(wheel_base)  # écart entre roues (m)
        self._v_lin_cmd = 0.0  # consigne vitesse linéaire
        self._v_ang_cmd = 0.0  # consigne vitesse angulaire

        # Stockage
        self._storage = Storage(storage_capacity)
        # Bras robotisé (injection) : on suppose implémentation d'ArmInterface
        self._arm = arm

        # Capteurs d'obstacle
        self._sensors = sensors or []

        # Planner
        self._planner = SimplePlanner()

        # État interne
        self._state = State.IDLE

        # Seuils
        self._obstacle_threshold = 0.3  # m : distance minimale d'arrêt
        self._battery_low_threshold = 20.0  # %
        # Logger
        self._logger = logging.getLogger(self.__class__.__name__)
        if not self._logger.handlers:
            handler = logging.StreamHandler()
            fmt = logging.Formatter("[%(levelname)s][%(name)s] %(message)s")
            handler.setFormatter(fmt)
            self._logger.addHandler(handler)
        self._logger.setLevel(logging.INFO)

        # Temps pour simulation (inutile si on ne gère pas dt ailleurs)
        self._last_time = time.time()

    # ---------- Méthodes bas-niveau moteurs / capteurs ----------

    def set_motor_speeds(self, left: float, right: float) -> None:
        """
        Commande les moteurs gauche/droite.
        En simulation simple, on convertit en v_lin_cmd et v_ang_cmd.
        """
        # v = (left + right)/2 ; omega = (right - left)/wheel_base
        self._v_lin_cmd = (left + right) / 2.0
        self._v_ang_cmd = (right - left) / self._wheel_base
        self._logger.info(f"Motors set: left={left:.2f}, right={right:.2f}")

    def stop_motors(self) -> None:
        """Arrête les moteurs immédiatement."""
        self._v_lin_cmd = 0.0
        self._v_ang_cmd = 0.0
        self._logger.info("Motors stopped.")

    def read_sensors(self) -> List[float]:
        """
        Lit chaque capteur et retourne la liste des distances.
        """
        dists = []
        for sensor in self._sensors:
            try:
                d = sensor.read()
                if d is None:
                    d = float('inf')
                dists.append(d)
            except Exception as e:
                self._logger.warning(f"Sensor read error: {e}")
        return dists

    # ---------- Consommation d'énergie ----------

    def _consume_for_motion(self, distance: float) -> None:
        """
        Consommation proportionnelle à la distance parcourue.
        Ici, un facteur arbitraire, à ajuster.
        """
        amount = abs(distance) * 0.5  # ex. 0.5% par mètre
        # On suppose que Robot a une méthode consume_energy(amount)
        self.consume_energy(amount)

    def _consume_for_rotation(self, angle: float) -> None:
        """
        Consommation proportionnelle à la rotation.
        """
        amount = abs(angle) * 2.0  # ex. 2% par radian
        self.consume_energy(amount)

    # ---------- Implémentation des méthodes abstraites Robot ----------

    def move(self, direction: str, distance: float) -> None:
        """
        Déplace le robot dans la direction donnée d'une certaine distance.
        direction: 'forward' ou 'backward' 
        """
        # Vérifier actif et batterie via base (on suppose Robot implémente _check_active ou similaire)
        try:
            self._check_active()  # ou: if not self.is_active: raise RuntimeError
        except AttributeError:
            # Si la classe de base ne définit pas _check_active, on vérifie is_active et battery_level
            if not self.is_active:
                raise RuntimeError("Robot inactive.")
            if getattr(self, "battery_level", None) is not None and self.battery_level <= 0:
                raise RuntimeError("Battery depleted.")

        if distance <= 0:
            return

        # Orientation: 0 rad = axe x positif. 'forward' avance dans orientation courante.
        # 'backward' avance dans orientation + pi.
        if direction not in ("forward", "backward"):
            raise ValueError("move direction must be 'forward' or 'backward'")

        angle = self._orientation if direction == "forward" else self._orientation + math.pi
        # Calcul de la nouvelle position
        dx = distance * math.cos(angle)
        dy = distance * math.sin(angle)
        new_pos = (self._position[0] + dx, self._position[1] + dy)

        # Consommation
        self._consume_for_motion(distance)

        # Mise à jour
        self._position = new_pos
        self._logger.info(f"Moved {direction} by {distance:.2f}m to {self._position}")

        # Vérifier batterie basse (on suppose propriété battery_level existe)
        if getattr(self, "battery_level", None) is not None and self.battery_level <= self._battery_low_threshold:
            self._logger.warning("Battery low after move.")
            # La machine à états ou appelant pourra gérer le retour ou mode économie

        self._last_time = time.time()

    def rotate(self, angle: float) -> None:
        """
        Tourne le robot de 'angle' radians (positif = sens anti-horaire).
        """
        try:
            self._check_active()
        except AttributeError:
            if not self.is_active:
                raise RuntimeError("Robot inactive.")
            if getattr(self, "battery_level", None) is not None and self.battery_level <= 0:
                raise RuntimeError("Battery depleted.")

        # Normaliser orientation
        new_ori = (self._orientation + angle) % (2 * math.pi)

        # Consommation
        self._consume_for_rotation(angle)

        self._orientation = new_ori
        self._logger.info(f"Rotated by {angle:.2f} rad to orientation {self._orientation:.2f}")

        if getattr(self, "battery_level", None) is not None and self.battery_level <= self._battery_low_threshold:
            self._logger.warning("Battery low after rotate.")

        self._last_time = time.time()

    def stop(self) -> None:
        """
        Arrête tout mouvement : ici on met les consignes à zéro.
        """
        try:
            self._check_active()
        except AttributeError:
            if not self.is_active:
                raise RuntimeError("Robot inactive.")
        self.stop_motors()
        self._logger.info("Robot stopped.")

    def status(self) -> str:
        """
        Renvoie une description textuelle brève du statut du robot mobile.
        On suppose que Robot de base a status_base() ou équivalent pour la partie générique.
        """
        base = ""
        if hasattr(super(), "status_base"):
            base = super().status_base()
        else:
            # fallback minimal
            base = f"Robot(ID={self.id}, Pos={self._position}, Ori={self._orientation:.2f})"
        storage_info = f"Storage: {self._storage.count()}/{self._storage._capacity}"
        state_info = f"State: {self._state}"
        return f"{base} | {storage_info} | {state_info}"

    # ---------- Navigation & évitement ----------

    def detect_obstacle(self) -> bool:
        """
        Vérifie si un obstacle est trop proche via les capteurs.
        Retourne True si au moins un capteur lit < threshold.
        """
        dists = self.read_sensors()
        for d in dists:
            if d is not None and d < self._obstacle_threshold:
                self._logger.info(f"Obstacle detected at distance {d:.2f}m")
                return True
        return False

    def avoid_obstacle(self) -> bool:
        """
        Stratégie simple de contournement :
        - Stoppe,
        - Recule un peu,
        - Tourne de 45 degrés,
        - Avance un peu,
        - Puis retourne True pour signaler manœuvre effectuée.
        """
        self._logger.info("Avoiding obstacle...")
        try:
            self.stop()
            self.move("backward", 0.2)
            self.rotate(math.pi / 4)
            self.move("forward", 0.3)
        except RuntimeError as e:
            self._logger.warning(f"Obstacle avoidance failed: {e}")
            return False
        self._logger.info("Obstacle avoidance maneuver done.")
        return True

    def navigate_to(self, target: Tuple[float, float]) -> bool:
        """
        Planifie et suit une trajectoire vers target (x, y).
        Retourne True si arrivé, False si interrompu (batterie faible ou obstacle infranchissable).
        """
        # Vérifie actif & batterie
        if not self.is_active or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
            self._logger.warning("Cannot navigate: inactive or battery low.")
            return False

        path = self._planner.compute_path(self._position, target)
        for waypoint in path:
            # Boucle jusqu'à atteindre le waypoint
            while True:
                # Obstacle ?
                if self.detect_obstacle():
                    ok = self.avoid_obstacle()
                    if not ok:
                        self._logger.warning("Cannot avoid obstacle, abort navigation.")
                        return False
                # Calcul vecteur vers waypoint
                dx = waypoint[0] - self._position[0]
                dy = waypoint[1] - self._position[1]
                dist = math.hypot(dx, dy)
                if dist < 0.05:
                    break  # waypoint atteint
                # Orientation souhaitée
                desired_angle = math.atan2(dy, dx)
                # Calcul de la différence d'angle dans [-π, π]
                angle_diff = (desired_angle - self._orientation + math.pi) % (2 * math.pi) - math.pi
                if abs(angle_diff) > 0.1:
                    # Tourne par pas
                    step = max(min(angle_diff, 0.3), -0.3)
                    try:
                        self.rotate(step)
                    except RuntimeError:
                        return False
                else:
                    # Avance vers waypoint
                    step_dist = min(dist, 0.2)
                    try:
                        self.move("forward", step_dist)
                    except RuntimeError:
                        return False
                # Vérifier batterie en cours de route
                if not self.is_active or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
                    self._logger.warning("Battery too low mid-navigation.")
                    return False
            # waypoint atteint
        self._logger.info(f"Arrived at target {target}")
        return True

    # ---------- Coordination avec le bras ----------

    def prepare_for_pickup(self, cube_pos: Tuple[float, float]) -> bool:
        """
        Positionne le robot pour que le bras puisse saisir le cube.
        cube_pos est la position absolue dans l'arène.
        """
        # Orientation vers le cube
        dx = cube_pos[0] - self._position[0]
        dy = cube_pos[1] - self._position[1]
        desired_angle = math.atan2(dy, dx)
        angle_diff = (desired_angle - self._orientation + math.pi) % (2 * math.pi) - math.pi
        try:
            self.rotate(angle_diff)
            # Approche jusqu'à distance de prise (ex. 0.3 m)
            dist_to_cube = math.hypot(dx, dy)
            approach_dist = max(dist_to_cube - 0.3, 0.0)
            if approach_dist > 0:
                self.move("forward", approach_dist)
        except RuntimeError:
            self._logger.warning("Failed to position for pickup.")
            return False
        # S'assurer que la plateforme est immobile
        self.stop()
        self._logger.info("Positioned for pickup.")
        return True

    def resume_after_pickup(self):
        """
        Réinitialise l'état pour reprendre navigation.
        """
        self._logger.info("Resuming navigation after pickup.")
        # Aucun code spécifique ; la machine à états reprendra la navigation.

    def add_cube_to_storage(self, cube_info: Any) -> bool:
        """
        Appelé après succès du pick par le bras.
        """
        success = self._storage.add(cube_info)
        if success:
            self._logger.info(f"Cube ajouté au stockage ({self._storage.count()}/{self._storage._capacity}).")
        else:
            self._logger.warning("Stockage plein, cube non ajouté.")
        return success

    def is_storage_full(self) -> bool:
        return self._storage.is_full()

    def return_to_sorting_zone(self, sort_zone_pos: Tuple[float, float]) -> bool:
        """
        Lance la navigation vers la zone de tri.
        """
        self._logger.info("Returning to sorting zone.")
        return self.navigate_to(sort_zone_pos)

    # ---------- Exemple de boucle de haute-niveau (orchestrateur) ----------

    def run_collection_cycle(self, collection_points: List[Tuple[float, float]], sort_zone_pos: Tuple[float, float]):
        """
        Exécution simplifiée d'un cycle de collecte :
        - Parcourt une liste de positions probables de cubes (ou zones)
        - Pour chaque position, navigue, prépare le pickup, appelle le bras, met à jour stockage.
        - Si stockage plein ou batterie faible, revient à sort_zone_pos, vide stockage, puis peut redémarrer.
        """
        self._state = State.NAVIGATING
        for pt in collection_points:
            # Vérifier actif et batterie
            if not self.is_active or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
                self._logger.info("Stopping collection cycle: inactive or battery low.")
                break

            # Navigation vers pt
            success_nav = self.navigate_to(pt)
            if not success_nav:
                self._logger.info(f"Cannot reach collection point {pt}. Skipping.")
                continue

            # Préparation pour pickup
            self._state = State.WAITING_FOR_ARM
            ok_pos = self.prepare_for_pickup(pt)
            if not ok_pos:
                continue

            # Demande au bras de détecter et saisir
            picked = False
            try:
                cube_rel = self._arm.detect_cube()  # position relative ou None
                if cube_rel is not None:
                    picked = self._arm.pick(cube_rel)
                else:
                    self._logger.info("Arm did not detect cube at expected location.")
            except Exception as e:
                self._logger.warning(f"Arm interface error: {e}")

            if picked:
                # Déposer dans le bac
                try:
                    placed = self._arm.place_into_storage()
                except Exception as e:
                    self._logger.warning(f"Arm placement error: {e}")
                    placed = False
                if placed:
                    self.add_cube_to_storage({"position": pt})
                else:
                    self._logger.warning("Failed to place cube into storage.")
            else:
                self._logger.info("Pickup failed or no cube.")

            # Vérifier stockage / batterie
            if self.is_storage_full() or (hasattr(self, "battery_level") and self.battery_level <= self._battery_low_threshold):
                self._logger.info("Storage full or battery low, returning to sort zone.")
                break

            self._state = State.NAVIGATING

        # Retour au tri
        self._state = State.RETURNING
        ret = self.return_to_sorting_zone(sort_zone_pos)
        if ret:
            self._logger.info("Arrived at sorting zone.")
            self._storage.clear()
            self._logger.info("Storage cleared after sorting.")
        else:
            self._logger.warning("Failed to return to sorting zone.")
        self._state = State.IDLE
        self._logger.info("Collection cycle completed.")
