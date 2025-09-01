import os
import pathlib
from ament_index_python.packages import get_package_share_directory
import yaml

import launch
from launch_ros.actions import Node

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'


def generate_launch_description():
    # Correctly reference the config file from the mounted /home/developer/config directory
    params_file = '/home/developer/config/imaging_source.yaml'

    with open(params_file, 'r') as f:
        parameters = yaml.safe_load(f)

    # Override 'camera_info_urls' to use the correct absolute path
    # Assuming camera_info_urls is relative to the config directory
    camera_info_path = os.path.join('/home/developer/config', parameters['camera_info_urls'])
    parameters['camera_info_urls'] = [camera_info_path]

    # override 'dynamic_parameters_yaml_url' to use the correct absolute path
    # Assuming dynamic_parameters_yaml_url is relative to the config directory
    dynamic_params_path = os.path.join('/home/developer/config', parameters['dynamic_parameters_yaml_url'])
    parameters['dynamic_parameters_yaml_url'] = dynamic_params_path

    example_package_node = Node(
        name='camera_driver_gv_example',
        package='camera_aravis2',
        executable='camera_driver_gv',
        output='screen',
        emulate_tty=True,
        parameters=[parameters],
        remappings=[
            ('/camera_driver_gv_example/vis/image_raw', '/camera/image_raw'),
            ('/camera_driver_gv_example/vis/camera_info', '/camera/camera_info')
        ]
    )
    return launch.LaunchDescription([example_package_node])
