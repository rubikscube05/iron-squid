import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction
)
from launch.substitutions import (
    PathJoinSubstitution,
    Command
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    pkg_octopus = FindPackageShare("octopus_description")

    pkg_octopus_path = get_package_share_directory(
        "octopus_description"
    )

    world_file = PathJoinSubstitution([
        pkg_octopus,
        "worlds",
        "oct.sdf"
    ])

    urdf_file = PathJoinSubstitution([
        pkg_octopus,
        "urdf",
        "tapered_octopus_arm.urdf"
    ])

    # ============================================================
    # ROBOT DESCRIPTION
    # ============================================================

    robot_description_file = os.path.join(
        pkg_octopus_path,
        "urdf",
        "tapered_octopus_arm.urdf"
    )

    robot_description = Command([
        "cat ",
        robot_description_file
    ])

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description
            }
        ]
    )

    # ============================================================
    # THREE SERVO TENDON CONTROLLER
    # ============================================================

    three_servo_controller = Node(
        package="octopus_description",
        executable="servo_controller.py",
        name="servo_controller",
        output="screen"
    )

    # ============================================================
    # NVIDIA GPU ENVIRONMENT
    # ============================================================

    nvidia_prime = SetEnvironmentVariable(
        name="__NV_PRIME_RENDER_OFFLOAD",
        value="1"
    )

    nvidia_glx = SetEnvironmentVariable(
        name="__GLX_VENDOR_LIBRARY_NAME",
        value="nvidia"
    )

    nvidia_vk = SetEnvironmentVariable(
        name="__VK_LAYER_NV_optimus",
        value="NVIDIA_only"
    )

    # ============================================================
    # GAZEBO RENDERER
    # ============================================================

    gazebo_renderer = SetEnvironmentVariable(
        name="GZ_SIM_RENDER_ENGINE",
        value="ogre2"
    )

    # ============================================================
    # GAZEBO SIM
    # ============================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                world_file
            ]
        }.items()
    )

    # ============================================================
    # SPAWN ROBOT
    # ============================================================

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "octopus_arm",

            "-file",
            urdf_file,

            "-x",
            "0.0",

            "-y",
            "0.0",

            "-z",
            "0.0",

            "-R",
            "0.0",

            "-P",
            "0.0",

            "-Y",
            "0.0"
        ],
        output="screen"
    )

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ============================================================
    # ARM CONTROLLER
    # ============================================================

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ============================================================
    # START ROBOT
    # ============================================================

    start_robot = TimerAction(
        period=3.0,
        actions=[
            spawn_robot
        ]
    )

    # ============================================================
    # START JOINT STATE BROADCASTER
    # ============================================================

    start_joint_state_broadcaster = TimerAction(
        period=6.0,
        actions=[
            joint_state_broadcaster
        ]
    )

    # ============================================================
    # START ARM CONTROLLER
    # ============================================================

    start_arm_controller = TimerAction(
        period=8.0,
        actions=[
            arm_controller
        ]
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([

        # --------------------------------------------------------
        # NVIDIA
        # --------------------------------------------------------

        nvidia_prime,
        nvidia_glx,
        nvidia_vk,

        # --------------------------------------------------------
        # GAZEBO RENDERER
        # --------------------------------------------------------

        gazebo_renderer,

        # --------------------------------------------------------
        # ROBOT STATE PUBLISHER
        # Start this BEFORE spawning the robot
        # --------------------------------------------------------

        robot_state_publisher,
        
        # -------------------------------------------------------- 
        # 3-servo tendon controller
        # -------------------------------------------------------- 
        
        three_servo_controller,

        # --------------------------------------------------------
        # GAZEBO
        # --------------------------------------------------------

        gazebo,

        # --------------------------------------------------------
        # SPAWN ROBOT AFTER GAZEBO STARTS
        # --------------------------------------------------------

        start_robot,

        # --------------------------------------------------------
        # CONTROLLERS
        # --------------------------------------------------------

        start_joint_state_broadcaster,
        start_arm_controller
    ])
