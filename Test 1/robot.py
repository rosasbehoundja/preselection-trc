from abc import ABC, abstractmethod
from typing import Union, Tuple, List, Any
import logging

class Robot(ABC):
    """
    Abstract base class representing a robot in a simulation environment.
    """

    # Sources d'énergie valides
    ENERGY_SOURCES = {"solar", "fossil_fuel", "electric"}

    def __init__(
        self,
        id: Union[int, str],
        name: str,
        position: Tuple[float, float],
        orientation: float,
        energy_source: str,
    ) -> None:
        # Validation types
        if not isinstance(id, (int, str)):
            raise TypeError("ID must be an integer or a string.")
        if not isinstance(name, str):
            raise TypeError("Name must be a string.")
        if (
            not isinstance(position, tuple)
            or len(position) != 2
            or not all(isinstance(coord, (int, float)) for coord in position)
        ):
            raise TypeError("Position must be a tuple of two numeric values (x, y).")
        if not isinstance(orientation, (int, float)):
            raise TypeError("Orientation must be a numeric value (in radians).")
        if energy_source.lower() not in self.ENERGY_SOURCES:
            raise ValueError(f"Energy source must be one of {self.ENERGY_SOURCES}.")

        self._id = id
        self._name = name
        self._position = (float(position[0]), float(position[1]))
        self._orientation = float(orientation)
        self._energy_source = energy_source.lower()

        # État général
        self._is_active = True
        # Niveaux d'énergie
        self._battery_level = 100.0  # en pourcentage
        # Optionnel : si modéliser un générateur distinct, on peut garder _generator_level séparé

        # Capteurs attachés
        self._sensors: List[Any] = []

        # Pour logging
        self._logger = logging.getLogger(self.__class__.__name__)
        if not self._logger.handlers:
            # Configuration basique si non configurée ailleurs
            handler = logging.StreamHandler()
            formatter = logging.Formatter("[%(levelname)s][%(name)s] %(message)s")
            handler.setFormatter(formatter)
            self._logger.addHandler(handler)
            self._logger.setLevel(logging.INFO)

        # Dernier temps de mise à jour (pour simulation temporelle si besoin)
        # import time
        # self._last_update = time.time()

    # ---------- Propriétés ----------

    @property
    def id(self) -> Union[int, str]:
        return self._id

    @property
    def name(self) -> str:
        return self._name

    @property
    def position(self) -> Tuple[float, float]:
        return self._position

    @position.setter
    def position(self, new_position: Tuple[float, float]) -> None:
        if (
            not isinstance(new_position, tuple)
            or len(new_position) != 2
            or not all(isinstance(coord, (int, float)) for coord in new_position)
        ):
            raise TypeError("Position must be a tuple of two numeric values (x, y).")
        self._position = (float(new_position[0]), float(new_position[1]))

    @property
    def orientation(self) -> float:
        return self._orientation

    @orientation.setter
    def orientation(self, new_orientation: float) -> None:
        if not isinstance(new_orientation, (int, float)):
            raise TypeError("Orientation must be numeric (radians).")
        # On peut normaliser dans [0, 2π)
        import math
        self._orientation = float(new_orientation) % (2 * math.pi)

    @property
    def energy_source(self) -> str:
        return self._energy_source

    @energy_source.setter
    def energy_source(self, src: str) -> None:
        if not isinstance(src, str) or src.lower() not in self.ENERGY_SOURCES:
            raise ValueError(f"Energy source must be one of {self.ENERGY_SOURCES}.")
        self._energy_source = src.lower()

    @property
    def battery_level(self) -> float:
        return self._battery_level

    @battery_level.setter
    def battery_level(self, new_level: float) -> None:
        if not isinstance(new_level, (int, float)):
            raise TypeError("Battery level must be numeric.")
        if not (0.0 <= new_level <= 100.0):
            raise ValueError("Battery level must be between 0 and 100.")
        self._battery_level = float(new_level)
        if self._battery_level <= 0:
            self._is_active = False
            self._logger.warning("Battery depleted: robot deactivated.")

    @property
    def is_active(self) -> bool:
        return self._is_active

    @is_active.setter
    def is_active(self, active: bool) -> None:
        if not isinstance(active, bool):
            raise TypeError("is_active must be a boolean.")
        self._is_active = active
        if not active:
            self._logger.info("Robot deactivated.")

    @property
    def sensors(self) -> List[Any]:
        # Retourne une copie pour éviter modification directe
        return list(self._sensors)

    # Pas de setter direct pour sensors, on fournit add/remove.
    
    # ---------- Gestion énergie ----------

    def _consume_energy(self, amount: float) -> None:
        """
        Réduit la batterie de `amount` (en pourcentage). 
        Si batterie < 0, désactive le robot.
        """
        if not isinstance(amount, (int, float)):
            raise TypeError("Energy consumption amount must be numeric.")
        new_level = self._battery_level - float(amount)
        self.battery_level = new_level  # passe par le setter pour gestion et logging

    def _recharge(self, amount: float) -> None:
        """
        Recharge la batterie de `amount` (en %), sans dépasser 100.
        Selon energy_source, on peut ajuster le taux.
        """
        if not isinstance(amount, (int, float)):
            raise TypeError("Recharge amount must be numeric.")
        new_level = min(100.0, self._battery_level + float(amount))
        self.battery_level = new_level
        self._logger.info(f"Battery recharged by {amount}%, new level: {self._battery_level:.1f}%")

    # ---------- Capteurs ----------

    def add_sensor(self, sensor: Any) -> None:
        if sensor not in self._sensors:
            self._sensors.append(sensor)
            self._logger.info(f"Sensor added: {sensor}")

    def remove_sensor(self, sensor: Any) -> None:
        if sensor in self._sensors:
            self._sensors.remove(sensor)
            self._logger.info(f"Sensor removed: {sensor}")

    # Méthode utilitaire
    def distance_to(self, other: Tuple[float, float]) -> float:
        import math
        x0, y0 = self._position
        x1, y1 = other
        return math.hypot(x1 - x0, y1 - y0)

    # Vérifie si robot peut agir
    def _check_active(self) -> None:
        if not self._is_active or self._battery_level <= 0:
            raise RuntimeError("Robot is inactive or battery depleted.")

    # ---------- Méthodes abstraites ----------

    @abstractmethod
    def move(self, direction: str, distance: float) -> None:
        """
        Déplacer le robot dans une direction ('forward','backward','left','right') d'une certaine distance.
        Doit appeler _check_active() et _consume_energy(...) selon la distance.
        """
        self._check_active()
        # La sous-classe implémente la modification de position selon orientation et direction.
        # Exemple pour consommation :
        # self._consume_energy(distance * facteur)
        raise NotImplementedError

    @abstractmethod
    def rotate(self, angle: float) -> None:
        """
        Tourner le robot de `angle` radians.
        Doit appeler _check_active() et _consume_energy(...) selon l'angle.
        """
        self._check_active()
        # La sous-classe met à jour self.orientation
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        """
        Arrêter tout mouvement en cours.
        """
        self._check_active()
        raise NotImplementedError

    @abstractmethod
    def status(self) -> str:
        """
        Retourne une description textuelle du statut courant.
        Peut appeler super().status_base() pour la partie générique.
        """
        raise NotImplementedError

    # Méthode de base pour status, utilisable par les sous-classes
    def status_base(self) -> str:
        """Partie générique du status, que les sous-classes peuvent réutiliser."""
        return (
            f"Robot(ID={self._id}, Name={self._name}, Pos={self._position}, "
            f"Ori={self._orientation:.2f} rad, Battery={self._battery_level:.1f}%, "
            f"Active={self._is_active})"
        )

    def __str__(self) -> str:
        return self.status_base()

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} {self.status_base()}>"

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Robot):
            return NotImplemented
        return self._id == other._id
