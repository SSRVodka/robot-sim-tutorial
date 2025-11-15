import os
import launch
import launch.launch_description_sources

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('diffdrive_car_desc'),
                    'launch',
                    'simulate.launch.py'
                )
            )
        )
    ])
