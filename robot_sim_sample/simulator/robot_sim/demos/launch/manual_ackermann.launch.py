import os
import launch
import launch.launch_description_sources

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> launch.LaunchDescription:

    # set env for simulator
    os.environ["MACHINE_TYPE"] = "JetRover_Acker"
    os.environ["LIDAR_TYPE"] = "A1"
    os.environ["CAMERA_TYPE"] = "HP60C"
    
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ackermann_car_desc'),
                    'launch',
                    'simulate.launch.py'
                )
            )
        )
    ])
