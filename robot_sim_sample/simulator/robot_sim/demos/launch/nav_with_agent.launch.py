import launch
from demos.template_nav_with_agent import create_launch_function


def generate_launch_description() -> launch.LaunchDescription:
    return create_launch_function()()
