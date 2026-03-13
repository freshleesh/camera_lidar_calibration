import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('extrinsic_calibration_by_hand_cpp')
    source_config_dir = '/ros2_ws/src/extrinsic_calibration_by_hand_cpp/config'
    general_file = os.path.join(source_config_dir, 'general.yaml')
    params_file = os.path.join(source_config_dir, 'params.yaml')
    intrinsic_file = os.path.join(source_config_dir, 'camera_intrinsic_calibration.yaml')
    extrinsic_file = os.path.join(source_config_dir, 'camera_extrinsic_calibration.yaml')

    node = Node(
        package='extrinsic_calibration_by_hand_cpp',
        executable='extrinsic_calibration_by_hand_cpp',
        name='extrinsic_calibration_by_hand',
        output='screen',
        parameters=[
            general_file,
            params_file,
            {
                'camera_intrinsic_yaml': intrinsic_file,
                'camera_extrinsic_yaml': extrinsic_file,
                'save_general_yaml': general_file,
                'save_params_yaml': params_file,
                'save_on_enter': True,
            },
        ],
    )

    return LaunchDescription([node])
