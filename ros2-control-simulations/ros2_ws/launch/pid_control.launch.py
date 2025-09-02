"""
Fichier launch principal pour le contrôle PID de turtlesim.
Démarre turtlesim et le contrôleur PID avec les paramètres.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Génère la description du launch."""

    # Chemin vers le package
    pkg_dir = get_package_share_directory('ros2_pid_turtle')

    # Arguments de lancement
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Utiliser le temps de simulation'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'pid_params.yaml'),
        description='Fichier de configuration des paramètres PID'
    )

    target_x_arg = DeclareLaunchArgument(
        'target_x',
        default_value='9.0',
        description='Position X cible'
    )

    target_y_arg = DeclareLaunchArgument(
        'target_y',
        default_value='9.0',
        description='Position Y cible'
    )

    target_theta_arg = DeclareLaunchArgument(
        'target_theta',
        default_value='0.0',
        description='Orientation cible (radians)'
    )

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description="Activer l'enregistrement des données"
    )

    # Node turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'background_r': 69,
            'background_g': 86,
            'background_b': 255
        }]
    )

    # Node contrôleur PID
    pid_controller_node = Node(
        package='ros2_pid_turtle',
        executable='control_node',
        name='pid_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_x': LaunchConfiguration('target_x'),
                'target_y': LaunchConfiguration('target_y'),
                'target_theta': LaunchConfiguration('target_theta'),
            }
        ]
    )

    # Node d'enregistrement des données (conditionnel)
    data_recorder_node = Node(
        package='ros2_pid_turtle',
        executable='data_recorder',
        name='data_recorder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_logging')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_file': '/tmp/turtle_control_data.csv'
        }]
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        config_file_arg,
        target_x_arg,
        target_y_arg,
        target_theta_arg,
        enable_logging_a
