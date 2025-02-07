from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('tamiyabot_controller'),
            'config',
            'tamiyabot_controllers.yaml',
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tamiyabot_controller', '--param-file', robot_controllers],
        remappings=[('tamiyabot_controller/tf_odometry', 'tf')],
    )
    

    return LaunchDescription(
        [
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
        ]
    )