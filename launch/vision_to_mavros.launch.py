#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_frame_id = LaunchConfiguration('target_frame_id')
    source_frame_id = LaunchConfiguration('source_frame_id')
    output_rate = LaunchConfiguration('output_rate')
    roll_cam = LaunchConfiguration('roll_cam')
    pitch_cam = LaunchConfiguration('pitch_cam')
    yaw_cam = LaunchConfiguration('yaw_cam')
    gamma_world = LaunchConfiguration('gamma_world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_frame_id',
            default_value='/camera_odom_frame',
            description='Target frame id (world frame)'
        ),
        DeclareLaunchArgument(
            'source_frame_id',
            default_value='/camera_link',
            description='Source frame id (camera frame)'
        ),
        DeclareLaunchArgument(
            'output_rate',
            default_value='30.0',
            description='Publishing rate in Hz'
        ),
        DeclareLaunchArgument(
            'roll_cam',
            default_value='0.0',
            description='Roll angle of the camera frame'
        ),
        DeclareLaunchArgument(
            'pitch_cam',
            default_value='0.0',
            description='Pitch angle of the camera frame'
        ),
        DeclareLaunchArgument(
            'yaw_cam',
            default_value='0.0',
            description='Yaw angle of the camera frame'
        ),
        DeclareLaunchArgument(
            'gamma_world',
            default_value='-1.5707963',
            description='Rotation of the world frame around Z'
        ),
        Node(
            package='vision_to_mavros',
            executable='vision_to_mavros_node',
            name='vision_to_mavros',
            output='screen',
            parameters=[{
                'target_frame_id': target_frame_id,
                'source_frame_id': source_frame_id,
                'output_rate': output_rate,
                'roll_cam': roll_cam,
                'pitch_cam': pitch_cam,
                'yaw_cam': yaw_cam,
                'gamma_world': gamma_world
            }],
            remappings=[('vision_pose', '/mavros/vision_pose/pose')]
        )
    ])
