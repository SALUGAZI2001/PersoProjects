"""
Fichier launch pour l\'analyse et la visualisation des performances.
Inclut l\'enregistrement de données et la génération de graphiques.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Génère la description du launch pour l\'analyse."""

    pkg_dir = get_package_share_directory('ros2_pid_turtle')

    # Arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='/tmp/turtle_experiment',
        description='Nom du fichier rosbag'
    )

    experiment_duration_arg = DeclareLaunchArgument(
        'experiment_duration',
        default_value='60',
        description='Durée de l\'expérience en secondes'
    )

    trajectory_type_arg = DeclareLaunchArgument(
        'trajectory_type',
        default_value='circle',
        description='Type de trajectoire à suivre'
    )

    # Node turtlesim
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    # Node contrôleur PID
    pid_controller_node = Node(
        package='ros2_pid_turtle',
        executable='control_node',
        name='pid_controller',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'pid_params.yaml')
        ]
    )

    # Node générateur de trajectoire
    trajectory_node = Node(
        package='ros2_pid_turtle',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[{
            'trajectory_type': LaunchConfiguration('trajectory_type'),
        }]
    )

    # Enregistrement rosbag
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/turtle1/pose',
            '/turtle1/cmd_vel', 
            '/control_data',
            '/reference_pose',
            '-o', LaunchConfiguration('bag_file')
        ],
        output='screen'
    )

    # Arrêt automatique après la durée spécifiée
    experiment_timer = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'sleep {LaunchConfiguration("experiment_duration")} && '
            'ros2 lifecycle set /pid_controller shutdown'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        bag_file_arg,
        experiment_duration_arg,
        trajectory_type_arg,

        # Nodes
        turtlesim_node,
        pid_controller_node,
        trajectory_node,
        rosbag_record,
        experiment_timer,
    ])