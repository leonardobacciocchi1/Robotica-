#!/usr/bin/env python3
"""
@file cooperative_localization_launch.py
@brief Launch file per il sistema di localizzazione cooperativa

Questo file di launch configura e avvia l'intero sistema di localizzazione
cooperativa centralizzato. Il sistema è composto da:
    1. Nodo EKF Centralizzato: Mantiene la stima congiunta di tutti i robot
    2. Nodi Robot: Simulano i robot pubblicando dati dal dataset MRCLAM
    3. Parametri Configurabili: Percorso dataset, velocità riproduzione, ecc.

Il launch file utilizza il dataset MRCLAM che contiene dati reali di:
- Odometria di ogni robot
- Misurazioni range-bearing verso landmark
- Misurazioni range-bearing tra robot
- Ground truth per valutazione

@author Lorenzo Nobili, Leonardo Bacciocchi, Cosmin Alessandro Minut
@date 2025
"""

from launch import LaunchDescription          
from launch_ros.actions import Node           
from launch.actions import DeclareLaunchArgument  
from launch.substitutions import LaunchConfiguration  
import os                                     
from ament_index_python.packages import get_package_share_directory  

def generate_launch_description():
    """
    Genera la descrizione del launch per il sistema di localizzazione cooperativa.

    Configura e avvia tutti i nodi necessari per il sistema:
    - Nodo EKF centralizzato per la stima congiunta
    - Nodi robot per la simulazione dei dati sensoriali
    - Parametri configurabili per dataset e timing

    Returns:
        LaunchDescription: Descrizione completa del sistema da eseguire
    """
    pkg_dir = get_package_share_directory('cooperative_localization')
    
    # NB: ARGOMENTI DI LAUNCH CONFIGURABILI !!!
    # I seguenti parametri di default possono essere modificati al momento di esecuzione del sistema

    # Percorso dataset
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='MRCLAM1/MRCLAM_Dataset1/',
        description='Percorso della directory del dataset MRCLAM'
    )

    # Velocità di riproduzione del dataset
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='2.0',
        description='Moltiplicatore per la velocità di riproduzione del dataset (1.0 = tempo reale)'
    )

    # Intervallo di stampa dello stato EKF
    print_interval_arg = DeclareLaunchArgument(
        'print_interval',
        default_value='3.0',  # Stampa ogni 3 secondi
        description='Intervallo per la stampa periodica dello stato EKF (in secondi)'
    )
    
    # NODO EKF CENTRALIZZATO
    # Il nodo principale che implementa l'algoritmo di localizzazione cooperativa
    centralized_node = Node(
        package='cooperative_localization',
        executable='coop_localization_node',    
        name='centralized_ekf_node',
        output='screen',                        # Mostra output nel terminale
        parameters=[{
            'num_robots': 5,                    # Numero di robot nel sistema
            'dataset_path': LaunchConfiguration('dataset_path'),
            'print_interval': LaunchConfiguration('print_interval'),
            'use_groundtruth_init': True        # Usa posizioni iniziali da ground truth
        }]
    )
    
    # NODI ROBOT
    # Ogni nodo robot simula un robot reale riproducendo i dati dal dataset
    # I nodi pubblicano su topic ROS2 che il nodo centralizzato riceve

    # Nodo Robot 1 - Pubblica dati del primo robot
    robot1_node = Node(
        package='cooperative_localization',
        executable='robot_node',               
        name='robot_1_node',
        output='screen',
        arguments=['1'],                       # ID del robot
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'playback_speed': LaunchConfiguration('playback_speed')
        }]
    )

    # Nodo Robot 2 - Pubblica dati del secondo robot
    robot2_node = Node(
        package='cooperative_localization',
        executable='robot_node',
        name='robot_2_node',
        output='screen',
        arguments=['2'],                       # ID del robot
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'playback_speed': LaunchConfiguration('playback_speed')
        }]
    )

    # Nodo Robot 3 - Pubblica dati del terzo robot
    robot3_node = Node(
        package='cooperative_localization',
        executable='robot_node',
        name='robot_3_node',
        output='screen',
        arguments=['3'],                       # ID del robot
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'playback_speed': LaunchConfiguration('playback_speed')
        }]
    )
    
    # Nodo Robot 4 - Pubblica dati del quarto robot
    robot4_node = Node(
        package='cooperative_localization',
        executable='robot_node',
        name='robot_4_node',
        output='screen',
        arguments=['4'],                       # ID del robot
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'playback_speed': LaunchConfiguration('playback_speed')
        }]
    )
    
    # Nodo Robot 5 - Pubblica dati del quinto robot
    robot5_node = Node(
        package='cooperative_localization',
        executable='robot_node',
        name='robot_5_node',
        output='screen',
        arguments=['5'],                       # ID del robot
        parameters=[{
            'dataset_path': LaunchConfiguration('dataset_path'),
            'playback_speed': LaunchConfiguration('playback_speed')
        }]
    )

    # DESCRIZIONE LAUNCH
    # Combina tutti gli argomenti ed i nodi
    return LaunchDescription([
        dataset_path_arg,
        playback_speed_arg,
        print_interval_arg,
        centralized_node,  
        robot1_node,       
        robot2_node,
        robot3_node,
        robot4_node,
        robot5_node
    ])


# ESEMPI DI UTILIZZO
# Per eseguire il sistema con parametri di default:
# ros2 launch cooperative_localization cooperative_localization_launch.py
#
# Per eseguire impostando una velocità di riproduzione (diversa da quella di default):
# ros2 launch cooperative_localization cooperative_localization_launch.py playback_speed:=1.0
#
# Per eseguire cambiando il percorso del dataset (da quello di default):
# ros2 launch cooperative_localization cooperative_localization_launch.py dataset_path:=/path/to/dataset/
#
# Per eseguire modificando l'intervallo di stampa dell'output (da quello di default):
# ros2 launch cooperative_localization cooperative_localization_launch.py print_interval:=5.0

# TOPIC ROS2 UTILIZZATI
# Input al nodo centralizzato:
# - /odom (nav_msgs/Odometry): Dati di odometria dai robot
# - /landmark (cooperative_localization/Measurement): Misurazioni verso landmark
# - /robot (cooperative_localization/Measurement): Misurazioni tra robot
# - /dataset_status (cooperative_localization/DatasetStatus): Stato completamento dataset
#
# Il sistema non pubblica su topic esterni ma stampa lo stato a console
