"""
Contrôleur PID simple pour le projet turtlesim.
Basé sur la librairie simple-pid avec adaptations.
"""

import time
from typing import Optional, Tuple

class PIDController:
    """
    Contrôleur PID simple pour le contrôle de position et d'orientation.
    """

    def __init__(self, 
                 kp: float = 1.0, 
                 ki: float = 0.0, 
                 kd: float = 0.0,
                 setpoint: float = 0.0,
                 output_limits: Optional[Tuple[float, float]] = None,
                 sample_time: float = 0.01):
        """
        Initialise le contrôleur PID.

        Args:
            kp: Gain proportionnel
            ki: Gain intégral  
            kd: Gain dérivé
            setpoint: Consigne à atteindre
            output_limits: Limites de sortie (min, max)
            sample_time: Période d'échantillonnage en secondes
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_limits = output_limits
        self.sample_time = sample_time

        # Variables internes
        self._last_time = None
        self._last_error = None
        self._integral = 0.0
        self._last_output = None

    def __call__(self, input_value: float, dt: Optional[float] = None) -> float:
        """
        Calcule la sortie PID pour l'entrée donnée.

        Args:
            input_value: Valeur actuelle du système
            dt: Pas de temps (optionnel)

        Returns:
            Sortie du contrôleur PID
        """
        current_time = time.time()

        # Calcul de l'erreur
        error = self.setpoint - input_value

        # Gestion du temps
        if dt is None:
            if self._last_time is None:
                dt = self.sample_time
            else:
                dt = current_time - self._last_time

        # Éviter la division par zéro
        if dt <= 0.0:
            dt = self.sample_time

        # Terme proportionnel
        proportional = self.kp * error

        # Terme intégral
        self._integral += error * dt
        integral = self.ki * self._integral

        # Terme dérivé
        if self._last_error is not None:
            derivative = self.kd * (error - self._last_error) / dt
        else:
            derivative = 0.0

        # Sortie PID
        output = proportional + integral + derivative

        # Application des limites
        if self.output_limits is not None:
            min_output, max_output = self.output_limits
            if output > max_output:
                output = max_output
                # Anti-windup: limiter l'intégrale
                self._integral -= (output - max_output) / (self.ki + 1e-6)
            elif output < min_output:
                output = min_output
                # Anti-windup: limiter l'intégrale
                self._integral -= (output - min_output) / (self.ki + 1e-6)

        # Sauvegarde pour la prochaine itération
        self._last_error = error
        self._last_time = current_time
        self._last_output = output

        return output

    def reset(self):
        """Reset tous les termes internes du PID."""
        self._last_time = None
        self._last_error = None
        self._integral = 0.0
        self._last_output = None

    def set_tunings(self, kp: float, ki: float, kd: float):
        """Modifie les gains du PID."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    @property
    def components(self) -> Tuple[float, float, float]:
        """Retourne les composantes P, I, D du dernier calcul."""
        if self._last_error is None:
            return 0.0, 0.0, 0.0

        p = self.kp * (self.setpoint - self._last_error)
        i = self.ki * self._integral
        d = self.kd * self._last_error if self._last_error else 0.0

        return p, i, d