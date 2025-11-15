
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from moveit_configs_utils import MoveItConfigsBuilder


PKG_NAME = "servo2c"
ROBOT_MOVEIT_CONFIG_PKG_NAME = "panda_moveit"


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name=ROBOT_MOVEIT_CONFIG_PKG_NAME)
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": "mujoco"
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    node_mtc_demo = Node(
        package=PKG_NAME,
        executable="mtc_node",
        name="mtc_demo_node",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[
            moveit_config.to_dict(),
        ],
        # prefix=['xterm -e gdb --args']
    )
    timer_mtc_demo = TimerAction(
        period=8.0,
        actions=[node_mtc_demo],
    )
    
    return LaunchDescription([
        timer_mtc_demo
    ])
