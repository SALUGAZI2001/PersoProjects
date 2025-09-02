"""
Node principal de contrôle PID pour turtlesim avec modèle bicyclette.
"""

import math
import time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray

from .pid_controller import PIDController
from .bicycle_model import BicycleModel, BicycleState, BicycleCommand

class TurtlePIDController(Node):
    """
    Node de contrôle PID pour turtlesim utilisant un modèle bicyclette.
    """

    def __init__(self):
        super().__init__('turtle_pid_controller')

        # Déclaration des paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pid_linear.kp', 2.0),
                ('pid_linear.ki', 0.0),
                ('pid_linear.kd', 0.1),
                ('pid_angular.kp', 4.0),
                ('pid_angular.ki', 0.0),
                ('pid_angular.kd', 0.2),
                ('target_x', 9.0),
                ('target_y', 9.0),
                ('target_theta', 0.0),
                ('control_frequency', 50.0),
                ('tolerance_position', 0.1),
                ('tolerance_angle', 0.05),
                ('bicycle.wheelbase', 1.0),
                ('bicycle.max_velocity', 3.0),
                ('bicycle.max_angular_velocity', 2.0)
            ]
        )

        # Récupération des paramètres
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_theta = self.get_parameter('target_theta').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.tolerance_pos = self.get_parameter('tolerance_position').value
        self.tolerance_angle = self.get_parameter('tolerance_angle').value

        # Initialisation du modèle bicyclette
        self.bicycle_model = BicycleModel(
            wheelbase=self.get_parameter('bicycle.wheelbase').value,
            max_velocity=self.get_parameter('bicycle.max_velocity').value,
            max_angular_velocity=self.get_parameter('bicycle.max_angular_velocity').value
        )

        # Initialisation des contrôleurs PID
        self.pid_distance = PIDController(
            kp=self.get_parameter('pid_linear.kp').value,
            ki=self.get_parameter('pid_linear.ki').value,
            kd=self.get_parameter('pid_linear.kd').value,
            output_limits=(0.0, self.bicycle_model.max_velocity)
        )

        self.pid_heading = PIDController(
            kp=self.get_parameter('pid_angular.kp').value,
            ki=self.get_parameter('pid_angular.ki').value,
            kd=self.get_parameter('pid_angular.kd').value,
            output_limits=(-self.bicycle_model.max_angular_velocity,
                          self.bicycle_model.max_angular_velocity)
        )

        # Variables d'état
        self.current_pose = None
        self.target_reached = False
        self.control_active = True

        # Historique pour l'analyse
        self.pose_history = []
        self.command_history = []
        self.error_history = []
        self.time_history = []
        self.start_time = time.time()

        # Publishers et Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publisher pour les données d'analyse
        self.data_pub = self.create_publisher(
            Float64MultiArray, '/control_data', 10)

        # Timer pour le contrôle
        self.control_timer = self.create_timer(
            1.0 / self.control_freq, self.control_callback)

        # Timer pour la publication des données
        self.data_timer = self.create_timer(
            0.1, self.publish_data)

        self.get_logger().info(f'Contrôleur PID initialisé')
        self.get_logger().info(f'Cible: x={self.target_x:.2f}, y={self.target_y:.2f}, θ={self.target_theta:.2f}')

    def pose_callback(self, msg: Pose):
        """Callback pour la réception de la pose de la tortue."""
        self.current_pose = msg

        # Enregistrer dans l'historique
        current_time = time.time() - self.start_time
        self.time_history.append(current_time)
        self.pose_history.append((msg.x, msg.y, msg.theta))

    def control_callback(self):
        """Callback principal de contrôle."""
        if self.current_pose is None:
            return

        if self.target_reached:
            # Arrêter la tortue si la cible est atteinte
            self.publish_stop_command()
            return

        # Calcul des erreurs
        error_x = self.target_x - self.current_pose.x
        error_y = self.target_y - self.current_pose.y
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Angle vers la cible
        angle_to_target = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(angle_to_target - self.current_pose.theta)

        # Vérifier si la cible est atteinte
        if (distance_error < self.tolerance_pos and 
            abs(angle_error) < self.tolerance_angle):
            self.target_reached = True
            self.get_logger().info('Cible atteinte!')
            self.publish_stop_command()
            return

        # Mise à jour des consignes PID
        self.pid_distance.setpoint = 0.0  # On veut réduire la distance à 0
        self.pid_heading.setpoint = 0.0   # On veut réduire l'erreur d'angle à 0

        # Calcul des commandes PID
        if distance_error > self.tolerance_pos:
            # Priorité à l'orientation si on est loin
            if abs(angle_error) > math.pi/4:  # 45 degrés
                velocity_cmd = 0.5  # Vitesse réduite pour tourner
                angular_cmd = self.pid_heading(-angle_error)
            else:
                # Avancer vers la cible
                velocity_cmd = self.pid_distance(-distance_error)
                angular_cmd = self.pid_heading(-angle_error)
        else:
            # Ajustement final de l'orientation
            final_angle_error = self.normalize_angle(self.target_theta - self.current_pose.theta)
            velocity_cmd = 0.0
            angular_cmd = self.pid_heading(-final_angle_error)

        # Créer la commande bicyclette
        bicycle_cmd = BicycleCommand(velocity_cmd, angular_cmd)

        # Valider et convertir en Twist
        if self.bicycle_model.validate_command(bicycle_cmd):
            twist_msg = self.bicycle_model.command_to_twist(bicycle_cmd)
            self.cmd_vel_pub.publish(twist_msg)

            # Enregistrer dans l'historique
            self.command_history.append((velocity_cmd, angular_cmd))
            self.error_history.append((distance_error, angle_error))
        else:
            self.get_logger().warn('Commande invalide générée par le PID')
            self.publish_stop_command()

    def publish_stop_command(self):
        """Publie une commande d'arrêt."""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def publish_data(self):
        """Publie les données pour l'analyse."""
        if self.current_pose is None:
            return

        # Préparer les données
        data_msg = Float64MultiArray()
        current_time = time.time() - self.start_time

        error_x = self.target_x - self.current_pose.x
        error_y = self.target_y - self.current_pose.y
        distance_error = math.sqrt(error_x**2 + error_y**2)

        angle_to_target = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(angle_to_target - self.current_pose.theta)

        # Format: [time, x, y, theta, target_x, target_y, target_theta, 
        #          distance_error, angle_error, cmd_vel, cmd_omega]
        data_msg.data = [
            current_time,
            float(self.current_pose.x),
            float(self.current_pose.y), 
            float(self.current_pose.theta),
            self.target_x,
            self.target_y,
            self.target_theta,
            distance_error,
            angle_error,
            0.0,  # cmd_vel (sera mis à jour)
            0.0   # cmd_omega (sera mis à jour)
        ]

        if self.command_history:
            data_msg.data[-2] = self.command_history[-1][0]  # cmd_vel
            data_msg.data[-1] = self.command_history[-1][1]  # cmd_omega

        self.data_pub.publish(data_msg)

    def normalize_angle(self, angle: float) -> float:
        """Normalise un angle entre -π et π."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def set_target(self, x: float, y: float, theta: float = 0.0):
        """Change la cible du contrôleur."""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta
        self.target_reached = False

        # Reset des PID
        self.pid_distance.reset()
        self.pid_heading.reset()

        self.get_logger().info(f'Nouvelle cible: x={x:.2f}, y={y:.2f}, θ={theta:.2f}')

    def get_performance_data(self) -> dict:
        """Retourne les données de performance pour l'analyse."""
        return {
            'time_history': self.time_history.copy(),
            'pose_history': self.pose_history.copy(),
            'command_history': self.command_history.copy(),
            'error_history': self.error_history.copy(),
            'target': (self.target_x, self.target_y, self.target_theta),
            'target_reached': self.target_reached
        }

def main(args=None):
    """Point d'entrée principal."""
    rclpy.init(args=args)

    controller = TurtlePIDController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Arrêt du contrôleur')
    finally:
        # Arrêter la tortue
        controller.publish_stop_command()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()