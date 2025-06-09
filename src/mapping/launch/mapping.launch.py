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
    world_name = LaunchConfiguration('world_name', default='nav_slam_world')
    holonomic_sim_pkg_dir = get_package_share_directory('holonomic_sim')
    pkg_share_dir = get_package_share_directory('mapping')
    model_path = os.path.join(holonomic_sim_pkg_dir, "models")
    autostart = LaunchConfiguration('autostart', default='true')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager", default='true')

    # ignition gazeboがモデルにアクセスできるように設定
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[os.path.join("/opt/ros/humble", "share"), ":" + model_path],
    )

    # ロボットをスポーンさせる設定
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity',
            'LidarRobo',
            '-name',
            'LidarRobo',
            # ロボットのsdfファイルを指定
            '-file',
            PathJoinSubstitution(
                [holonomic_sim_pkg_dir, "models", "LidarRobo", "model.sdf"]
            ),
            # ロボットの位置を指定
            '-allow_renaming',
            'true',
            '-x',
            '0.4',
            '-y',
            '0.4',
            '-z',
            '0.075',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # フィールドをスポーンさせる設定
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        # フィールドのsdfファイルを指定
        arguments=[
            '-file',
            PathJoinSubstitution(
                [holonomic_sim_pkg_dir, "models", "field", "model.sdf"]
            ),
            '-allow_renaming',
            'false',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ワールドのsdfファイルを設定(worldタグのあるsdfファイル)
    world = os.path.join(
        holonomic_sim_pkg_dir, "models", "worlds", "holonomic_test.sdf"
    )

    # ignition gazeboの起動設定
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
        launch_arguments=[('ign_args', [' -r -v 3 ' + world])],
    )

    # ros_ign_bridgeの起動設定
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                # brigdeの設定ファイルを指定
                'config_file': os.path.join(pkg_share_dir, 'config', 'slam.yaml'),
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
                'qos_overrides./odom.publisher.durability': 'transient_local',
            },
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen',
    )

    # ロボットのsdfファイルのパスを取得
    sdf = os.path.join(holonomic_sim_pkg_dir, 'models', 'LidarRobo', 'model.sdf')

    # xacroでsdfファイルをurdfに変換
    doc = xacro.parse(open(sdf))
    xacro.process_doc(doc)

    # robot_state_publsherの起動設定
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True, 'robot_description': doc.toxml()}],
    )  # type: ignore

    # rviz2の設定フィルのパスを取得
    rviz_config_dir = os.path.join(pkg_share_dir, 'config', 'mapping.rviz')

    # rviz2の起動設定
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    slam_params_file = os.path.join(pkg_share_dir, 'config', 'slam_params.yaml')

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
            ignition_spawn_world,
            ign_gz,
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description='If true, use simulated clock',
            ),
            DeclareLaunchArgument(
                'world_name', default_value=world_name, description='World name'
            ),
            bridge,
            robot_state_publisher,
            rviz2,
            start_async_slam_toolbox_node,
            configure_event,
            activate_event,
        ]
    )
