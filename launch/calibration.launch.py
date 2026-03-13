import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    source_config_dir = '/ros2_ws/src/camera_lidar_calibration/config'
    general_file = os.path.join(source_config_dir, 'general.yaml')
    params_file = os.path.join(source_config_dir, 'camera_extrinsic_calibration.yaml')
    intrinsic_file = os.path.join(source_config_dir, 'camera_intrinsic_calibration.yaml')

    node = Node(
        package='camera_lidar_calibration',
        executable='camera_lidar_calibration',
        name='extrinsic_calibration_by_hand',
        output='screen',
        parameters=[
            general_file,
            params_file,
            {
                'camera_intrinsic_yaml': intrinsic_file,
                'save_general_yaml': general_file,
                'save_params_yaml': params_file,
                'save_on_enter': True,
            },
        ],
    )

    return LaunchDescription([node])
