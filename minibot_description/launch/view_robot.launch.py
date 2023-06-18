# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, EnvironmentVariable
# from launch.conditions import IfCondition
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     urdf_path = PathJoinSubstitution(
#         [FindPackageShare("minibot_description"), "urdf", "robot.urdf.xacro"]
#     )

#     rviz_config_path = PathJoinSubstitution(
#         [FindPackageShare('minibot_description'), 'rviz', 'description.rviz']
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             name='urdf', 
#             default_value=urdf_path,
#             description='URDF path'
#         ),
        
#         DeclareLaunchArgument(
#             name='publish_joints', 
#             default_value='true',
#             description='Launch joint_states_publisher'
#         ),

#         DeclareLaunchArgument(
#             name='rviz', 
#             default_value='false',
#             description='Run rviz'
#         ),

#         DeclareLaunchArgument(
#             name='use_sim_time', 
#             default_value='false',
#             description='Use simulation time'
#         ),

#         Node(
#             package='joint_state_publisher',
#             executable='joint_state_publisher',
#             name='joint_state_publisher',
#             condition=IfCondition(LaunchConfiguration("publish_joints"))
#             # parameters=[
#             #     {'use_sim_time': LaunchConfiguration('use_sim_time')}
#             # ] #since galactic use_sim_time gets passed somewhere and rejects this when defined from launch file
#         ),

#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             parameters=[
#                 {
#                     'use_sim_time': LaunchConfiguration('use_sim_time'),
#                     'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
#                 }
#             ]
#         ),

#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', rviz_config_path],
#             condition=IfCondition(LaunchConfiguration("rviz")),
#             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
#         )
#     ])

# #sources: 
# #https://navigation.ros.org/setup_guides/index.html#
# #https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
# #https://github.com/ros2/rclcpp/issues/940

import os
from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('minibot_description'),
        'rviz',
        'view_robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        on_exit=Shutdown()
    )

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'ignore_timestamp': False,
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('minibot_description'),
                        'urdf',
                        'robot.urdf.xacro',
                    ]),
                ]),
        }]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen'
    )

    ld.add_action(rviz_node)
    ld.add_action(rsp_node)
    ld.add_action(jsp_gui_node)

    return ld