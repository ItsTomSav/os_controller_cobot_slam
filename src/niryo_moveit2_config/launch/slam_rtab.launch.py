import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Percorsi dei pacchetti
    niryo_moveit_config_pkg = get_package_share_directory('niryo_moveit2_config')
    # Launch di base: robot + gazebo + moveit + rviz
    niryo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(niryo_moveit_config_pkg, 'launch', 'niryo_slam.launch.py')
        )
    )

    # Collegamento: world → map
    static_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_map',
        output="log",
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        parameters=[
            {'use_sim_time': True}
        ],
    )

    # # Collegamento: map → odom
    # static_map_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_odom',
    #     output="log",
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

    # # Collegamento: odom → base_link
    # static_odom_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_odom_base',
    #     output="log",
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )
    

    # Nodo RTAB-Map
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # --- MODIFICA FONDAMENTALE: Usare 'base_link' come frame di riferimento ---
            # Questo fa sì che la mappa e la posa stimata siano relative alla base
            # fissa del robot, rendendo più facile ottenere la posa dell'end-effector
            # nel frame della mappa (map -> base_link -> ... -> end_effector)
            'frame_id': 'base_link',
            'odom_frame_id': 'base_link',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,           # Non voglio usare il laser scanner
            'approx_sync': True,               # Sync approx images - Sincronizza gli input (RGB, Depth, Info)
            'sync_queue_size': 30,             # Dimensione della coda di sincronizzazione

            # --- Parametri Odometria/SLAM (calcolati internamente) ---
            # 'visual_odometry': True,           # Abilita Visual Odometry  --Non esiste
            'RGBD/NeighborLinkRefining': 'false',  # Ottimizza i link tra keyframe
            'Reg/Force3DoF': 'false',            # Non voglio solo stima di X, Y, Yaw, ma anche Z e Roll e Pitch -- Forza SLAM 6DOF
            'RGBD/LinearUpdate': '0.0',         # Distanza minima tra keyframe   #prima avevo 0.05
            'RGBD/AngularUpdate': '0.0',        # Angolo minimo tra keyframe     #prima avevo 0.05
            'RGBD/MaxDepth': '3.5',              # Profondità massima per le feature
            'Mem/IncrementalMemory': 'true',
            'Mem/DepthCompressionFormat': '.png',
            'Vis/MinInliers': '15',

            # --- Parametri Mappatura/Memoria ---
            'RGBD/OptimizeMaxError': '3.0',      # Migliora stabilità loop closure - Soglia errore per ottimizzazione grafo
            'RGBD/ProximityBySpace': 'false',
            #'Mem/UseOdomFeatures': 'true',       # Riusa i feature dell'odometria - Irrilevante in nodo singolo, avrei dovuto avere il nodo odometria
            'Mem/ImagePreDecimation': '4',       # Decimazione immagini, utile per performance (1=no, 2=metà, 4=un quarto)
            'Rtabmap/DetectionRate': '1',
            'Grid/FromDepth': 'true',            # Genera occupancy grid dalla profondità
            'Grid/RayTracing': 'false',           # Usa ray tracing per generare la griglia
            'Grid/3D': 'true',                   # Usa la griglia 3D per la mappatura
            'Grid/CellSize': '0.1',

            # --- Parametri Registrazione/Loop Closure ---
            'Reg/Strategy': '0',                 # Visual + ICP (Iterative Closest Point)
            'Optimizer/Slam2D': 'false',         # Impedisce di forzare la mappa a rimanere 2D.

            # --- Parametri TF ---
            'wait_for_transform': 0.3,          # Attendi le trasformate TF all'avvio
            'tf_delay': 0.05,
            'tf_tolerance': 0.1,
            
            # --- Parametri Keypoint ---
            'Kp/MaxFeatures': '-1',            # Disabilita loop closure basata su feature visive

            #Quality Of Service QoS
            'qos_image': 2,                 # QoS per immagini (0=Best Effort, 1=Reliable, 2=Transient Local)
            'qos_camera_info': 2,
        }],
        arguments=['-d'],                       # Per cancellare il database a ogni avvio
        remappings=[
            ('rgb/image', '/gazebo_depth_camera/image_raw'),
            ('depth/image', '/gazebo_depth_camera/depth/image_raw'),
            ('rgb/camera_info', '/gazebo_depth_camera/camera_info'),
        ]
    )

    # Nodo RTAB-Map Viz (interfaccia grafica per visualizzazione)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # --- MODIFICA: Usare 'map' come frame fisso per la visualizzazione ---
            # Questo permette di vedere la mappa costruirsi rispetto a un riferimento globale
            'frame_id': 'base_link',
            'odom_frame_id': 'base_link',

            # 'subscribe_depth': True,
            'subscribe_rgbd': False,
            'subscribe_scan': False,

            'approx_sync': True,
            'sync_queue_size': 30,

            #Quality Of Service QoS
            'qos_image': 2,
            'qos_camera_info': 2,
        }],
        remappings=[
            ('rgb/image', '/gazebo_depth_camera/image_raw'),
            ('depth/image', '/gazebo_depth_camera/depth/image_raw'),
            ('rgb/camera_info', '/gazebo_depth_camera/camera_info'),
        ]
    )

    # Avviare rtabmap_node dopo 20 secondi
    delayed_rtabmap = TimerAction(
        period=20.0,
        actions=[rtabmap_node, rtabmap_viz_node]
    )

    return LaunchDescription([
        niryo_launch,

        static_world_map,
        # static_map_odom,
        # static_odom_base,

        delayed_rtabmap,
    ])

