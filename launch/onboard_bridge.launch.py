# launch file for onboard_bridge 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    params_file_path = os.path.join(get_package_share_directory(
      'onboard_bridge'), 'config', 'onboard_bridge_params.yaml')
    
    logging_level = 'INFO'

    onboard_bridge = Node(
        package = 'onboard_bridge',
        executable = 'bridge',
        name = 'bridge',
        output = 'screen',
        emulate_tty = True,
        arguments = [
            '--ros-args',
            '--params-file',
            params_file_path,
            '--log-level',
            logging_level
        ]
    )

    return LaunchDescription([
        onboard_bridge,
    ])