import launch
import launch.launch_description_sources
import launch_ros

from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart

from ament_index_python.packages import get_package_share_directory

import os
from typing import Callable


def create_launch_function(
        ENV_PKG_NAME: str = 'diffdrive_car_desc',
        NAV_CONFIG: str = 'nav2_params.diffdrive_car.yaml',
        NAV_MAP_CONFIG: str = 'room.yaml'
    ) -> Callable[[], launch.LaunchDescription]:
    """
    Create a launch function from template (with agent control and navigation)
    :param ENV_PKG_NAME: The name of a robot description package which manage a simulation environment
    :param NAV_CONFIG: The navigation configuration file in controllers/nav2c
    :param NAV_MAP_CONFIG: The navigation map file in controllers/nav2c
    """
    def generate_launch_description_template() -> launch.LaunchDescription:
        AGENT_PKG_NAME = "agent2c"
        NAV_CONTROL_PKG_NAME = "nav2c"
        sim_env_pkg_shared_dir = get_package_share_directory(ENV_PKG_NAME)
        sim_nav_pkg_shared_dir = get_package_share_directory(NAV_CONTROL_PKG_NAME)
        bringup_official_dir = get_package_share_directory('nav2_bringup')

        rviz_config_dir = os.path.join(
            bringup_official_dir, 'rviz', 'nav2_default_view.rviz')
        
        use_sim_time = launch.substitutions.LaunchConfiguration(
            'use_sim_time', default='true')
        map_yaml_path = launch.substitutions.LaunchConfiguration(
            'map', default=os.path.join(sim_nav_pkg_shared_dir, 'maps', NAV_MAP_CONFIG))
        nav2_param_path = launch.substitutions.LaunchConfiguration(
            'params_file', default=os.path.join(sim_nav_pkg_shared_dir, 'config', NAV_CONFIG))

        # set env for simulator
        os.environ["MACHINE_TYPE"] = "JetRover_Acker"
        os.environ["LIDAR_TYPE"] = "A1"
        os.environ["CAMERA_TYPE"] = "HP60C"

        action_sim_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(sim_env_pkg_shared_dir, 'launch', 'simulate.launch.py')
            )
        )

        action_nav_base_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(bringup_official_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path,
                # for humble ONLY
                # 'yaml_filename': map_yaml_path,
            }.items(),
        )
        action_map_display_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_map_display_node',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )

        action_sim_nav_node = launch_ros.actions.Node(
            package=NAV_CONTROL_PKG_NAME,
            executable='nav2c',
            name='nav2controller_node',
            output='screen'
        )

        action_pose_manager_node = launch_ros.actions.Node(
            package=NAV_CONTROL_PKG_NAME,
            executable='pose_manager',
            name='pose_manager_node',
            output='screen'
        )

        action_agent_control_node = launch_ros.actions.Node(
            package=AGENT_PKG_NAME,
            executable='agent_control_sim',
            name='agent_control_sim_node',
            output='screen'
        )

        # Create timed launch sequence using cascading TimerActions

        nav_base_timer = TimerAction(
            period=5.0,
            actions=[action_nav_base_launch]
        )
        map_display_timer = TimerAction(
            period=10.0,
            actions=[action_map_display_node]
        )
        pose_manager_timer = TimerAction(
            period=15.0,
            actions=[action_pose_manager_node]
        )
        sim_nav_timer = TimerAction(
            period=25.0,
            actions=[action_sim_nav_node]
        )
        
        # Start action_agent_control_node after 18 seconds (same as pose_manager, but use event handler)
        agent_control_timer = RegisterEventHandler(
            OnProcessStart(
                target_action=action_pose_manager_node,
                on_start=[action_agent_control_node]
            )
        )
        
        return launch.LaunchDescription([
            launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                                description='Use simulation (Gazebo) clock if true'),
            launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                                description='Full path to map file to load'),
            launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                                description='Full path to param file to load'),
            # TODO
            # action_sim_launch -(间隔 3 秒)-> action_nav_base_launch -(间隔 10 秒)-> action_sim_nav_node -(间隔 5 秒)-> action_pose_manager_node -> action_agent_control_node -> action_map_display_node
            
            # Start the sequence
            action_sim_launch,  # Starts immediately
            nav_base_timer,     # Starts after 5 seconds
            map_display_timer,  # Starts after 10 seconds

            pose_manager_timer, # Starts after 15 seconds (3 + 10 + 5)

            sim_nav_timer,      # Starts after 25 seconds (3 + 10)
            
            # Register event handlers for the last node
            agent_control_timer,
        ])
    
    return generate_launch_description_template
