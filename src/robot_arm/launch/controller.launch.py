import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction


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

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "pause": "true",
            "extra_gazebo_args": "--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so",
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
                )
            ]
        )
    )

    # Nodes
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_the_robot",
        arguments=[
            "-entity",
            "robot_arm",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-topic",
            "robot_description",
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
            "/robot_arm/controller_manager",
        ],
        parameters=[controllers_config_path],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        remappings=[("/joint_states", "/robot_arm/joint_states")],
        output="screen",
    )

    rqt_reconfigure_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_reconfigure",
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        # namespace="robot_arm",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/robot_arm/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            declare_x,
            declare_y,
            declare_z,
            gazebo_server,
            gazebo_client,
            robot_state_publisher_node,
            TimerAction(period=5.0, actions=[spawn_robot_node]),
            TimerAction(
                period=2.0,
                actions=[
                    joint_state_broadcaster_spawner,
                    controller_spawner_node,
                ],
            ),
            # spawn_robot_node,
            # joint_state_broadcaster_spawner,
            # controller_spawner_node,
            rqt_reconfigure_node,
        ]
    )
