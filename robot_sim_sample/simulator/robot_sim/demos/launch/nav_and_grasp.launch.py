import launch
from demos.template_nav_with_agent import create_launch_function


def generate_launch_description() -> launch.LaunchDescription:
    return create_launch_function(
        ENV_PKG_NAME='ackermann_car_desc',
        NAV_CONFIG='nav2_params.ackermann_car.yaml',
        NAV_MAP_CONFIG='room.yaml')()
