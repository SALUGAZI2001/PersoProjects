"""
Générateur de trajectoires pour le contrôle de turtlesim.
"""

import math
import time
from typing import List, Tuple, Generator

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class TrajectoryGenerator(Node):
    """
    Générateur de trajectoires de référence pour turtlesim.
    """

    def __init__(self):
        super().__init__('trajectory_generator')

        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('trajectory_type', 'circle'),  # circle, square, figure8, waypoints
                ('center_x', 5.5),
                ('center_y', 5.5),
                ('radius', 2.0),
                ('period', 20.0),  # Période en secondes
                ('publish_rate', 10.0),  # Hz
                ('square_size', 3.0),
            ]
        )

        # Récupération des paramètres
        self.traj_type = self.get_parameter('trajectory_type').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.radius = self.get_parameter('radius').value
        self.period = self.get_parameter('period').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.square_size = self.get_parameter('square_size').value

        # Publisher
        self.ref_pub = self.create_publisher(
            PoseStamped, '/reference_pose', 10)

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_reference)

        # Variables
        self.start_time = time.time()
        self.waypoints = self.generate_waypoints()
        self.current_waypoint_idx = 0

        self.get_logger().info(f'Générateur de trajectoire initialisé: {self.traj_type}')

    def generate_waypoints(self) -> List[Tuple[float, float, float]]:
        """Génère les points de passage selon le type de trajectoire."""
        waypoints = []

        if self.traj_type == 'circle':
            # Trajectoire circulaire
            n_points = int(self.publish_rate * self.period)
            for i in range(n_points):
                t = 2 * math.pi * i / n_points
                x = self.center_x + self.radius * math.cos(t)
                y = self.center_y + self.radius * math.sin(t)
                theta = t + math.pi/2  # Tangent à la trajectoire
                waypoints.append((x, y, theta))

        elif self.traj_type == 'square':
            # Trajectoire carrée
            half_size = self.square_size / 2
            corners = [
                (self.center_x - half_size, self.center_y - half_size, 0.0),
                (self.center_x + half_size, self.center_y - half_size, math.pi/2),
                (self.center_x + half_size, self.center_y + half_size, math.pi),
                (self.center_x - half_size, self.center_y + half_size, -math.pi/2),
            ]

            # Interpolation entre les coins
            points_per_side = int(self.publish_rate * self.period / 4)
            for i in range(4):
                start = corners[i]
                end = corners[(i + 1) % 4]

                for j in range(points_per_side):
                    alpha = j / points_per_side
                    x = start[0] + alpha * (end[0] - start[0])
                    y = start[1] + alpha * (end[1] - start[1])
                    theta = end[2]  # Orientation vers le prochain coin
                    waypoints.append((x, y, theta))

        elif self.traj_type == 'figure8':
            # Trajectoire en forme de 8
            n_points = int(self.publish_rate * self.period)
            for i in range(n_points):
                t = 2 * math.pi * i / n_points
                # Paramètres pour le 8
                x = self.center_x + self.radius * math.sin(t)
                y = self.center_y + self.radius * math.sin(t) * math.cos(t)
                # Calcul de l'orientation tangente
                dx_dt = self.radius * math.cos(t)
                dy_dt = self.radius * (math.cos(2*t))
                theta = math.atan2(dy_dt, dx_dt)
                waypoints.append((x, y, theta))

        elif self.traj_type == 'waypoints':
            # Points de passage prédéfinis
            predefined_waypoints = [
                (2.0, 2.0, 0.0),
                (9.0, 2.0, math.pi/2),
                (9.0, 9.0, math.pi),
                (2.0, 9.0, -math.pi/2),
                (5.5, 5.5, 0.0),
            ]

            # Interpolation entre les waypoints
            points_per_segment = int(self.publish_rate * self.period / len(predefined_waypoints))
            for i in range(len(predefined_waypoints)):
                start = predefined_waypoints[i]
                end = predefined_waypoints[(i + 1) % len(predefined_waypoints)]

                for j in range(points_per_segment):
                    alpha = j / points_per_segment
                    x = start[0] + alpha * (end[0] - start[0])
                    y = start[1] + alpha * (end[1] - start[1])
                    theta = math.atan2(end[1] - start[1], end[0] - start[0])
                    waypoints.append((x, y, theta))

        else:
            # Point fixe par défaut
            waypoints = [(self.center_x, self.center_y, 0.0)]

        return waypoints

    def publish_reference(self):
        """Publie la pose de référence courante."""
        if not self.waypoints:
            return

        # Pose courante dans la trajectoire
        current_pose = self.waypoints[self.current_waypoint_idx]

        # Création du message
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = current_pose[0]
        pose_msg.pose.position.y = current_pose[1]
        pose_msg.pose.position.z = 0.0

        # Conversion angle en quaternion (rotation autour de z)
        theta = current_pose[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)

        self.ref_pub.publish(pose_msg)

        # Passage au waypoint suivant
        self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)

    def get_trajectory_info(self) -> dict:
        """Retourne les informations sur la trajectoire."""
        return {
            'type': self.traj_type,
            'waypoints': self.waypoints,
            'total_points': len(self.waypoints),
            'period': self.period,
            'current_index': self.current_waypoint_idx
        }

def main(args=None):
    """Point d'entrée principal."""
    rclpy.init(args=args)

    trajectory_generator = TrajectoryGenerator()

    try:
        rclpy.spin(trajectory_generator)
    except KeyboardInterrupt:
        trajectory_generator.get_logger().info('Arrêt du générateur de trajectoire')
    finally:
        trajectory_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()