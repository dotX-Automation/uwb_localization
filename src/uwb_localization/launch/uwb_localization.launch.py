# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

import sys
params_holder_path = '/home/neo/workspace/src/params_holder'
sys.path.insert(1, params_holder_path)
import update_yaml
update_yaml.update_params()

config = os.path.join(params_holder_path, 'config/params_holder.yaml')

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uwb_localization',
            executable='uwb_localization',
            name='uwb_localization',
            output='screen',
            parameters=[config]
        ),
    ])
