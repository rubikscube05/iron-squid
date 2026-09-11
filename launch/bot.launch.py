import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # PACKAGE PATHS
    # ============================================================

    package_share = FindPackageShare("octopus_description")

    urdf_file = PathJoinSubstitution([
        package_share,
        "urdf",
        "squid.urdf"
    ])

    world_file = PathJoinSubstitution([
        package_share,
        "worlds",
        "oct.sdf"
    ])

    # ============================================================
    # GAZEBO RESOURCE PATH
    # ============================================================

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value="/home/vaibhav/doc_oct/src"
    )

    # ============================================================
    # NVIDIA / GAZEBO SETTINGS
    # ============================================================

    nvidia_offload = SetEnvironmentVariable(
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

    gazebo_renderer = SetEnvironmentVariable(
        name="GZ_SIM_RENDER_ENGINE",
        value="ogre2"
    )

    # ============================================================
    # ROBOT DESCRIPTION
    # ============================================================

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="cat"),
            " ",
            urdf_file
        ])
    }

    # ============================================================
    # ROBOT STATE PUBLISHER
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": True
            }
        ]
    )

    # ============================================================
    # GAZEBO HARMONIC
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
            "gz_args": ["-r ", world_file]
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
            "octopus",

            "-file",
            urdf_file,

            "-x",
            "0",

            "-y",
            "0",

            "-z",
            "0.8"
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
    # ARM CONTROLLERS
    # ============================================================

    arm1_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm1_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    arm2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm2_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    arm3_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm3_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        output="screen"
    )

    # ============================================================
    # SERVO CONTROLLER NODE
    # ============================================================

    servo_controller = Node(
        package="octopus_description",
        executable="servo_controller.py",
        name="servo_controller",
        output="screen"
    )

    # ============================================================
    # DELAY CONTROLLERS
    # ============================================================

    controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster,
            arm1_controller,
            arm2_controller,
            arm3_controller,
            servo_controller
        ]
    )

    # ============================================================
    # LAUNCH DESCRIPTION
    # ============================================================

    return LaunchDescription([
        gazebo_resource_path,
        nvidia_offload,
        nvidia_glx,
        nvidia_vk,
        gazebo_renderer,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        controllers,
    ])
