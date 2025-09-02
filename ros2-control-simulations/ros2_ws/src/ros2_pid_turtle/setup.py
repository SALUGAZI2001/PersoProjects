import os
from glob import glob
from setuptools import setup

package_name = 'ros2_pid_turtle'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installation des fichiers launch
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Installation des fichiers de configuration
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # Installation des scripts
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nom Étudiant',
    maintainer_email='etudiant@universite.fr',
    description='Contrôleur PID avec modèle bicyclette pour turtlesim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Node principal de contrôle
            'control_node = ros2_pid_turtle.control_node:main',
            # Générateur de trajectoires
            'trajectory_generator = ros2_pid_turtle.trajectory_generator:main',
            # Script d'analyse
            'analyze_performance = ros2_pid_turtle.analyze_performance:main',
        ],
    },
)