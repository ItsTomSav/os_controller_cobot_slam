import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Mappature URDF/Xacro
    launch_arguments = {
        "use_fake_hardware": "false",
        "dof": "6",
    }

    # Configurazione MoveIt
    moveit_config = (
        MoveItConfigsBuilder("niryo_ned2", package_name="niryo_moveit2_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp"],
            default_planning_pipeline="chomp"
        )
        .sensors_3d(
            file_path=os.path.join(
                get_package_share_directory("niryo_moveit2_config"),
                "config",
                "sensors_3d.yaml",
            )
        )
        .to_moveit_configs()
    )

    # Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            ],
    )

    # Percorsi
    pkg_share = FindPackageShare("niryo_moveit2_config")
    gazebo_pkg_share = FindPackageShare("gazebo_ros")
    rviz_base = LaunchConfiguration("rviz_config")

    # RViz
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    rviz_config = PathJoinSubstitution([pkg_share, "config", rviz_base])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # Static TF: world → base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
        parameters=[
            {'use_sim_time': True}  # Aggiunto dopo aver avuto problemi con RTAB
        ],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("niryo_moveit2_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": True},   # Aggiunto dopo aver avuto problemi con RTAB
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Controller spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["niryo_arm_controller", "-c", "/controller_manager"],
    )

    # -------- AGGIUNTA NUOVO CONTROLLER --------
    op_space_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["niryo_op_space_controller", "-c", "/controller_manager", "--inactive"],
    )


    # -------- PARTE NUOVA GAZEBO --------
    # INIZIO

    # Path al file .world                           #aggiunta per il world
    world_path = PathJoinSubstitution([
        FindPackageShare("worlds_niryo"),  # nome del pacchetto
        "worlds",                          # sottocartella dove è salvato il world
        "pick_and_place_workplace_cube.world"
    ])

    # Include: avvia Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world_path}.items()   #aggiunta per il world, (aggiunta anche virgola sopra)
    )

    # Include: spawna il robot in Gazebo (risorse + description + pose)
    #spawn_robot = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution([pkg_share, "launch", "rsp.launch.py"])
    #    )
    #)
    spawn_entity_node = ExecuteProcess(
        cmd=[
            "ros2", "run", "gazebo_ros", "spawn_entity.py",
            "-entity", "niryo_ned2",
            "-topic", "robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.8"  #cambiato
        ],
        output="screen"
    )

    # FINE

    return LaunchDescription([
        rviz_config_arg,
        gazebo,
        spawn_entity_node,
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        op_space_controller_spawner, # Nuovo controller aggiunto
    ])

