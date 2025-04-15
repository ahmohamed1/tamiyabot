import os
from pathlib import Path
from os import pathsep
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    is_sim = LaunchConfiguration('is_sim', default=True)
    tamiyabot_description = get_package_share_directory("tamiyabot_description")

    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        tamiyabot_description, "urdf", "tamiyabot_1.urdf.xacro"
                                        ),
                                      description="Absolute path to robot xacro file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    world_path = PathJoinSubstitution([
            tamiyabot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )


    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(tamiyabot_description).parent.resolve())
            ]
        )
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            get_package_share_directory("tamiyabot_description"),
            "urdf",
            "tamiyabot_model.urdf.xacro",
        ),
        description="URDF file to publish",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('tamiyabot_controller'),
            'config',
            'tamiyabot_controllers.yaml',
        ]
    )
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "tamiyabot",
                   "-z", '0.05',
                   "x", "-0.5"],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Modify the controller spawner to ensure odometry publishing
    tamiyabot_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tamiyabot_controller',
                   '--param-file',
                   robot_controllers,
                   '--controller-ros-args',
                   '-r /tamiyabot_controller/tf_odometry:=/tf',
                 ],
    )

    # Wrap the controller spawner in a TimerAction for delay
    delayed_controller_spawner = TimerAction(
        period=7.0,  # 5 seconds delay
        actions=[
            tamiyabot_steering_controller_spawner
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/tamiyabot_controller/odometry', '/odom']
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel','/tamiyabot_controller/reference']
    )

    
    return LaunchDescription([
        gz_ros2_bridge,
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        # Launch gazebo environment
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[delayed_controller_spawner],
            )
        ),
        gz_spawn_entity,
        odom_relay,       # Add odom relay
        cmd_vel_relay,    # Add cmd_vel relay

        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'description_format',
            default_value='urdf',
            description='Robot description format to use, urdf or sdf'),
    ])