#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    state_pub_node = Node(
        package='my_DDS_pkg',
        executable='robot_001_state_pub',
        name='robot_001_state_pub',
        output='screen',
        parameters=[{
            'robot_id': 'robot_001',
            'wifi_interface': 'wlan0',
            'wifi_update_rate': 1.0,
            'battery_type': '12V',
            'full_voltage': 12.6,
            'empty_voltage': 5.0,
            'warning_voltage': 6.5
        }]
    )

    speed_node = Node(
        package='my_DDS_pkg',
        executable='robot_001_speed',
        name='robot_001_speed',
        output='screen',
        parameters=[{
            'publish_rate': 10.0,
            'topic_name': '/robot/velocity'
        }]
    )

    camera_node = Node(
        package='my_DDS_pkg',
        executable='robot_001_camera',
        name='robot_001_camera',
        output='screen',
        parameters=[{
            'camera_index': 0,
            'frame_width': 640,
            'frame_height': 480,
            'fps': 5,
            'save_path': '/root/pictures'
        }]
    )
    delayed_camera = TimerAction(period=5.0, actions=[camera_node])

    task_node = Node(
        package='my_DDS_pkg',
        executable='robot_001_task',
        name='robot_001_task',
        output='screen',
        parameters=[{
            'robot_id': 'robot_001',
            'task_id': 1
        }]
    )
    delayed_task = TimerAction(period=10.0, actions=[task_node])

    return LaunchDescription([
        state_pub_node,
        speed_node,
        delayed_camera,
        delayed_task
    ])
