import os
from os import pathsep

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    tamiyabot_description = get_package_share_directory("tamiyabot_description")
    tamiyabot_description_prefix = get_package_prefix("tamiyabot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Correctly set the GAZEBO_MODEL_PATH
    model_path = os.path.join(tamiyabot_description, "models")
    model_path += pathsep + os.path.join(tamiyabot_description_prefix, "share")

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            tamiyabot_description, "urdf", "tamiyabot.urdf.xacro"
        ),
        description="Absolute path to robot urdf file",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="cone_track",
        description="Name of the Gazebo world to load",
    )

    # Construct world path substitution
    # world_path = PathJoinSubstitution([
    #     tamiyabot_description,
    #     "worlds",
    #     LaunchConfiguration("world_name"),
    # ])

    world_path = os.path.join(
        get_package_share_directory("tamiyabot_description"),
        "worlds",
        "cone_track.world",
    )
    
    # Define the robot description
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    # Nodes and launch configurations
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tamiyabot_controller"],
    )

    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path}.items(),
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "tamiyabot", "-topic", "robot_description"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("tamiyabot_description"),
                "rviz",
                "tamiyabot.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            env_var,
            model_arg,
            world_name_arg,
            start_gazebo_server,
            start_gazebo_client,
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_node,
            spawn_robot,
            rviz_node,
        ]
    )
