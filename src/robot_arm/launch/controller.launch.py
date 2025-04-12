import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Paths to required files
    robot_arm_description_path = os.path.join(
        get_package_share_directory("robot_arm"), "urdf", "robot_arm.xacro"
    )

    robot_description = {
        "robot_description": Command(["xacro ", robot_arm_description_path]),
        "use_sim_time": True,
    }

    controllers_config_path = os.path.join(
        get_package_share_directory("robot_arm"), "config", "controller.yaml"
    )

    # Launch arguments for robot's initial position
    declare_x = DeclareLaunchArgument("x", default_value="0")
    declare_y = DeclareLaunchArgument("y", default_value="0")
    declare_z = DeclareLaunchArgument("z", default_value="0")

    # Launch Gazebo (Ignition)
    gazebo = ExecuteProcess(cmd=["ign", "gazebo", "-r", "empty.sdf"], output="screen")

    # Nodes
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_the_robot",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "robot_arm",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-allow_renaming",
            "true",
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    get_package_share_directory("robot_arm"),
                    "config",
                    "bridge_config.yaml",
                ),
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        arguments=[
            "joint1_position_controller",
            "joint2_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[controllers_config_path],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    # rqt_reconfigure_node = Node(
    #     package="rqt_gui",
    #     executable="rqt_gui",
    #     name="rqt_reconfigure",
    #     output="screen",
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            declare_x,
            declare_y,
            declare_z,
            gazebo,
            robot_state_publisher_node,
            bridge,
            spawn_robot_node,
            joint_state_broadcaster_spawner,
            controller_spawner_node,
            # rqt_reconfigure_node,
        ]
    )
