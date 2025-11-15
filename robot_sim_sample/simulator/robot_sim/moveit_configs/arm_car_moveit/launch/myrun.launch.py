import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

PKG_NAME = "arm_car_moveit"

ROBOT_NAME = "voxelsky_car"
WORLD_NAME = "custom_room.world"
GAZEBO_BRIDGE_CONFIG_NAME = "gz_bridge.yaml"

def generate_launch_description():
    PKG_SHARED_DIR = get_package_share_directory(PKG_NAME)
    asset_car_desc_shared_dir = get_package_share_directory('arm_car_desc')
    asset_world_shared_dir = get_package_share_directory('home_demo_world')
    asset_models_shared_dir = get_package_share_directory('common_models')

    world_path = os.path.join(asset_world_shared_dir, "world", WORLD_NAME)
    gz_bridge_conf_path = os.path.join(asset_car_desc_shared_dir, "config", GAZEBO_BRIDGE_CONFIG_NAME)

    moveit_config = (
        # default package name is ${robot_name}_moveit_configs
        MoveItConfigsBuilder(ROBOT_NAME, package_name=PKG_NAME)
        .robot_description(file_path="config/voxelsky_car.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines("ompl", ["ompl"])
        .moveit_cpp(
            file_path=os.path.join(
                PKG_SHARED_DIR,
                "config",
                "xhw_config.yaml",
            )
        )
        .to_moveit_configs()
    )

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="moveit2c_node",
        package="moveit2c",
        executable="car_arm_controller",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    main_timer = TimerAction(
        period=20.0,
        actions=[moveit_cpp_node]
    )

    # RViz
    rviz_config_file = os.path.join(
        PKG_SHARED_DIR,
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            # {"use_sim_time": True}
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        PKG_SHARED_DIR,
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_group_controller",
        "gripper_group_controller",
        "joint_state_broadcaster",
        "vbot_ackermann_steering_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]
    
    ######### keyboard teleop #########

    # @ref: https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/adv-teleop/#stampedunstamped-twist
    action_unstamp_cmd_vel = Node(
        package='twist_stamper',
        executable='twist_stamper',
        # NOTE: twist_unstamper is similar
        # executable='twist_unstamper',
        remappings=[('/cmd_vel_in','/cmd_vel'),
                    ('/cmd_vel_out','/cmd_vel_stamped')]
    )

    ######## Gazebo Sim Env ##########

    action_set_gz_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', ':'.join([
            os.path.join(asset_models_shared_dir, "models"),
            # Note: 'package://xxx' work with RVIZ but not for GAZEBO;
            # However '$(find xxx)' work with GAZEBO but not for RVIZ;
            # 'file://$(find xxx)' works with both RVIZ and GAZEBO.
            # So here fix model find path starts with package://xxx
            os.path.join(asset_car_desc_shared_dir, ".."),
        ])
    )
    # @ref: https://blog.csdn.net/ZhangRelay/article/details/141561700
    # @ref: https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/
    # ros2 launch ros_gz_sim gz_sim.launch.py
    # NOTE: delegate to another Launch Description
    action_launch_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # launch_arguments=[('gz_args', world_path)]
        # launch_arguments={'gz_args': ['-v4 ', world_path]}.items()
        # NOTE: 如果使用 ros2_control controllers，则 -r 立即开始模拟是必要的！否则会因为 timeout 启动失败
        # launch_arguments={'gz_args': ['-r -v4 ', world_path]}.items()
        # NOTE: 如果模型中存在 mimic 关节，则必须使用新的引擎，否则会出现错误
        # @ref: https://github.com/ros-controls/gz_ros2_control/issues/340
        launch_arguments={'gz_args': [
            '-r -v4 ', '--physics-engine gz-physics-bullet-featherstone-plugin ', world_path]}.items()
    )
    # @ref: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim
    # @ref: https://robotics.stackexchange.com/questions/113022/ros2-jazzy-gz-ignition-harmonic-question-about-description-pkg
    action_spawn_robot_in_world = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', ROBOT_NAME,
            # '-allow_renaming', 'true',
            # '-x', "0.0",
            # '-y', "0.0",
            # '-z', "0.0",
            # '-R', "0.0",
            # '-P', "0.0",
            # '-Y', "0.0"
        ]
    )
    # @ref: https://ibrahimmansur4.medium.com/creating-a-custom-differential-drive-robot-in-gazebo-sim-af939cd8bb97
    action_gz2ros_topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_bridge_conf_path}',
        ],
    )
    # NOTE: comment out because it cannot rename topic. see gz_bridge.yaml
    # @ref: https://gazebosim.org/docs/latest/migrating_gazebo_classic_ros2_packages/
    # action_gz_cam_image_raw_bridge = launch_ros.actions.Node(
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=['/camera/image_raw'],
    #     output='screen',
    # )

    return LaunchDescription(
        [
            action_set_gz_env,
            
            # static_tf,
            robot_state_publisher,

            action_gz2ros_topic_bridge,
            action_launch_gz,
            action_spawn_robot_in_world,

            rviz_node,
            # moveit_cpp_node,
            ros2_control_node,
        ]
        + load_controllers
        + [action_unstamp_cmd_vel]
        + [main_timer,]
    )