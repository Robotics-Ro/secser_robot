from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

import os

def generate_launch_description():
    prefix = DeclareLaunchArgument("prefix", default_value="")
    lidar_model = DeclareLaunchArgument("lidar_model", default_value="hokuyo")
    lidar_port_name = DeclareLaunchArgument("lidar_port_name", default_value="/dev/ttyLidar")
    lidar_baudrate = DeclareLaunchArgument("lidar_baudrate", default_value="57600")
    robot_port_name = DeclareLaunchArgument("robot_port_name", default_value="/dev/ttySTM32")


    sensors_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'sensors.launch.py']
    )

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("secser_base"), "config", "ekf.yaml"]
    )

    custom_robot_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_bringup'), 'launch', 'custom_robot.launch.py']
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('linorobot2_description'),
            'urdf/robot/mecanum.urdf.xacro',
        ]),
        ' is_sim:=', 'false',
        ' lidar_model:=', LaunchConfiguration('lidar_model'),
        ' port_name:=', LaunchConfiguration('robot_port_name'),
        ' baudrate:=', LaunchConfiguration('robot_baudrate'),
        ' prefix:=', LaunchConfiguration('prefix'),

    ])

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("secser_base"), "config", "ekf.yaml"]
    )

    robot_controllers = PathJoinSubstitution([
            FindPackageShare('secser_bringup'),
            "config",
            "minibot_controllers.yaml"
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            robot_controllers
            ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    load_minibot_io_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'minibot_io_controller'],
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='joy', 
            default_value='false',
            description='Use Joystick'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ),
    ])

        prefix,
        lidar_model,
        lidar_port_name,
        lidar_baudrate,
        robot_port_name,
        robot_baudrate,
        upload_robot,
        control_node,
        ydlidar_params_declare,
        ydlidar_driver_node,
    ])