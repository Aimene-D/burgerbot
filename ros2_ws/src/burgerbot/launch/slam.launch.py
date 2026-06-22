from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = FindPackageShare('burgerbot')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # slam_toolbox in Jazzy is a managed lifecycle node. It must be transitioned
    # configure -> activate or it stays unconfigured: no params, no /scan
    # subscription, no map->odom TF, no /map. We auto-drive those transitions
    # here (use_lifecycle_manager:=false → no external bond/manager required).
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[
            PathJoinSubstitution([pkg, 'config', 'slam_toolbox.yaml']),
            {'use_lifecycle_manager': False, 'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    # Wait for the slam_toolbox process to start before sending configure.
    # A bare EmitEvent at the top level fires before the node is ready,
    # so the configure service call is silently lost → node stays unconfigured
    # → no /scan subscription → no map→odom TF → scan rotates with robot.
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=slam_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ))
            ]
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[slam_toolbox] configured, activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'),

        slam_node,
        configure_event,
        activate_event,
    ])
