#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the respeaker_node
        Node(
            package='respeaker_ros',
            executable='respeaker_node',
            name='respeaker_node',
            output='screen',
            parameters=[
                {'sensor_frame_id': 'respeaker_base'},
                {'speech_prefetch': 0.5},
                {'update_period_s': 0.1},
                {'main_channel': 0},
                {'speech_continuation': 0.5},
                {'speech_max_duration': 7.0},
                {'speech_min_duration': 0.1},
                {'doa_xy_offset': 0.0},
                {'doa_yaw_offset': 90.0}
            ]
        ),
        
        # Launch the doa_listener node
        Node(
            package='respeaker_ros',
            executable='doa_listener',
            name='doa_listener',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ]
        )
    ])
