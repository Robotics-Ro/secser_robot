from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    use_sim_time = True

    joy_launch_path = PathJoinSubstitution(
        [FindPackageShare('secser_bringup'), 'launch', 'joy_teleop.launch.py']
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("secser_base"), "config", "ekf.yaml"]
    )

    world_path = PathJoinSubstitution(
        [FindPackageShare("linorobot2_gazebo"), "worlds", "gas_station.world"]
    )

    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('linorobot2_description'), 'launch', 'description.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='world', 
            default_value=world_path,
            description='Gazebo world'
        ),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "secser"] # 기존 linobot2 폴더라고 생각해서 secser 폴더로 바꿈
        ),

        Node(
            package='linorobot2_gazebo',
            executable='command_timeout.py',
            name='command_timeout'
        ), # command_timeout.py 사용할시 install에 copy된 파일에 chmod +x 해줘야함, 시간 관련 파이썬 파일이 아닐까 함 

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            remappings=[("odometry/filtered", "odom")]
        ), # odom 세팅

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'publish_joints': 'false',
            }.items()
        ), # 잘 모르겠음

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_path),
        )
    ])

#sources: 
#https://navigation.ros.org/setup_guides/index.html#
#https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
#https://github.com/ros2/rclcpp/issues/940
