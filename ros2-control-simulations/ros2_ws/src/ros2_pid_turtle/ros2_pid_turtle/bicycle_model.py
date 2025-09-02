"""
Modèle cinématique bicyclette pour le contrôle de turtlesim.
"""

import math
from typing import Tuple, NamedTuple
from geometry_msgs.msg import Twist

class BicycleState(NamedTuple):
    """État du modèle bicyclette."""
    x: float      # Position X
    y: float      # Position Y  
    theta: float  # Orientation (rad)
    v: float      # Vitesse linéaire

class BicycleCommand(NamedTuple):
    """Commande pour le modèle bicyclette."""
    v_cmd: float    # Vitesse linéaire désirée
    omega: float    # Vitesse angulaire désirée

class BicycleModel:
    """
    Modèle cinématique bicyclette pour turtlesim.
    """

    def __init__(self, 
                 wheelbase: float = 1.0,
                 max_steering_angle: float = math.pi/3,
                 max_velocity: float = 5.0,
                 max_angular_velocity: float = 2.0):
        """
        Initialise le modèle bicyclette.

        Args:
            wheelbase: Empattement du véhicule (L)
            max_steering_angle: Angle de braquage maximum
            max_velocity: Vitesse linéaire maximum
            max_angular_velocity: Vitesse angulaire maximum
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity

    def compute_steering_angle(self, v: float, omega: float) -> float:
        """
        Calcule l'angle de braquage à partir de v et omega.

        Relation: omega = (v/L) * tan(delta)
        Donc: delta = arctan(L * omega / v)

        Args:
            v: Vitesse linéaire
            omega: Vitesse angulaire

        Returns:
            Angle de braquage (rad)
        """
        if abs(v) < 1e-3:  # Éviter la division par zéro
            return 0.0

        delta = math.atan(self.wheelbase * omega / v)

        # Limiter l'angle de braquage
        delta = max(-self.max_steering_angle, 
                   min(self.max_steering_angle, delta))

        return delta

    def compute_angular_velocity(self, v: float, delta: float) -> float:
        """
        Calcule la vitesse angulaire à partir de v et delta.

        Args:
            v: Vitesse linéaire
            delta: Angle de braquage

        Returns:
            Vitesse angulaire (rad/s)
        """
        omega = (v / self.wheelbase) * math.tan(delta)

        # Limiter la vitesse angulaire
        omega = max(-self.max_angular_velocity,
                   min(self.max_angular_velocity, omega))

        return omega

    def state_update(self, 
                    state: BicycleState, 
                    command: BicycleCommand, 
                    dt: float) -> BicycleState:
        """
        Met à jour l'état du modèle bicyclette.

        Args:
            state: État actuel
            command: Commande à appliquer
            dt: Pas de temps

        Returns:
            Nouvel état
        """
        # Limiter les commandes
        v_cmd = max(-self.max_velocity, 
                   min(self.max_velocity, command.v_cmd))
        omega = max(-self.max_angular_velocity,
                   min(self.max_angular_velocity, command.omega))

        # Dynamique simple de vitesse (filtre du premier ordre)
        tau = 0.1  # Constante de temps
        v_new = state.v + (v_cmd - state.v) * dt / tau

        # Cinématique bicyclette
        x_new = state.x + v_new * math.cos(state.theta) * dt
        y_new = state.y + v_new * math.sin(state.theta) * dt
        theta_new = state.theta + omega * dt

        # Normaliser l'angle
        theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))

        return BicycleState(x_new, y_new, theta_new, v_new)

    def command_to_twist(self, command: BicycleCommand) -> Twist:
        """
        Convertit une commande bicyclette en message Twist ROS2.

        Args:
            command: Commande bicyclette

        Returns:
            Message Twist pour ROS2
        """
        twist = Twist()

        # Limiter les commandes
        twist.linear.x = max(-self.max_velocity,
                           min(self.max_velocity, command.v_cmd))
        twist.angular.z = max(-self.max_angular_velocity,
                            min(self.max_angular_velocity, command.omega))

        return twist

    def validate_command(self, command: BicycleCommand) -> bool:
        """
        Valide qu'une commande respecte les contraintes physiques.

        Args:
            command: Commande à valider

        Returns:
            True si la commande est valide
        """
        if abs(command.v_cmd) > self.max_velocity:
            return False

        if abs(command.omega) > self.max_angular_velocity:
            return False

        # Vérifier la cohérence cinématique
        if abs(command.v_cmd) > 1e-3:
            required_delta = abs(self.compute_steering_angle(
                command.v_cmd, command.omega))
            if required_delta > self.max_steering_angle:
                return False

        return True