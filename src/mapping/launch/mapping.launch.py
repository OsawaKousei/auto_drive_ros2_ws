import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='test_world')
    # get the package directory
    holonomic_sim_pkg_dir = get_package_share_directory('holonomic_sim')
    pkg_shared_dir = get_package_share_directory('mapping')
    # get the model path
    model_path = os.path.join(holonomic_sim_pkg_dir, "models")
    autostart = LaunchConfiguration('autostart', default='true')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager", default='true')

    # set the environment variable for the gazebo resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join("/opt/ros/humble", "share"), ":" + model_path],
    )

    # spawn the robot
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity',
            'HolonomicRobo',
            '-name',
            'HolonomicRobo',
            # specify the robot description topic
            '-topic',
            'robot_description',
            '-allow_renaming',
            'true',
            # set robot position
            '-x',
            '0.5',
            '-y',
            '0.5',
            '-z',
            '1.0',
        ],
    )

    # spawn the field
    ignition_spawn_field = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        # specify the field path
        arguments=[
            '-file',
            PathJoinSubstitution([model_path, "field", "model.sdf"]),
            '-allow_renaming',
            'false',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.0',
        ],
    )

    # get the world file path
    world = os.path.join(model_path, "worlds", "holonomic_test.sdf")

    # launch the gazebo
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('ros_ign_gazebo'),
                    'launch',
                    'ign_gazebo.launch.py',
                )
            ]
        ),
        # specify the world file
        launch_arguments=[('ign_args', [' -r -v 1 ' + world])],
    )

    # launch the message bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                # specify the configuration file
                'config_file': os.path.join(
                    holonomic_sim_pkg_dir, 'config', 'holonomic_test.yaml'
                ),
                # set the QoS profile
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./odom.publisher.durability': 'transient_local',
            },
            {'use_sim_time': use_sim_time},
        ],
        # remap the topic
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen',
    )

    # get the urdf file path
    urdf = os.path.join(model_path, 'HolonomicUrdf', 'model.xacro')
    # expand the xacro file
    robot_desc = xacro.process_file(urdf).toxml()

    # launch the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        arguments=[robot_desc],
        # specify the robot description
        parameters=[
            {
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    # get the rviz config file path
    rviz_config_dir = os.path.join(pkg_shared_dir, 'config', 'mapping.rviz')

    # launch the rviz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # specify the config file
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    slam_params_file = os.path.join(pkg_shared_dir, 'params', 'slam_params.yaml')

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': use_sim_time,
            },
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            start_async_slam_toolbox_node
                        ),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    return LaunchDescription(
        [
            ign_resource_path,
            ignition_spawn_entity,
            ignition_spawn_field,
            ign_gz,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description='If true, use simulated clock',
            ),
            bridge,
            DeclareLaunchArgument(
                'world_name', default_value=world_name, description='World name'
            ),
            robot_state_publisher,
            rviz2,
            start_async_slam_toolbox_node,
            configure_event,
            activate_event,
        ]
    )
