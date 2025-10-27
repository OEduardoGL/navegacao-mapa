from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def _launch_planner(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    cmd = [
        'python3',
        '-m',
        'tb3_map_dstar.dstar_planner_node',
        '--ros-args',
        '-p', f'map_yaml:={lc("map_yaml")}',
        '-p', f'start_x:={lc("start_x")}',
        '-p', f'start_y:={lc("start_y")}',
        '-p', f'goal_x:={lc("goal_x")}',
        '-p', f'goal_y:={lc("goal_y")}',
        '-p', f'occupied_threshold:={lc("occupied_threshold")}',
        '-p', f'allow_diagonal:={lc("allow_diagonal")}',
        '-p', f'safety_radius:={lc("safety_radius")}',
        '-p', f'simplify_distance:={lc("simplify_distance")}',
    ]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def _launch_follower(context, *args, **kwargs):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    cmd = [
        'python3',
        '-m',
        'tb3_map_dstar.path_follower_node',
        '--ros-args',
        '-p', 'path_topic:=planned_path',
        '-p', 'odom_topic:=/odom',
        '-p', 'cmd_vel_topic:=/cmd_vel',
        '-p', 'max_linear_speed:=0.3',
        '-p', 'max_angular_speed:=1.2',
        '-p', 'goal_tolerance:=0.5',
        '-p', 'waypoint_tolerance:=0.35',
        '-p', 'linear_gain:=0.8',
        '-p', 'angular_gain:=2.0',
        '-p', f'lookahead:={lc("lookahead")}',
    ]
    return [ExecuteProcess(cmd=cmd, output='screen')]


def generate_launch_description():
    tb3_pkg = get_package_share_directory('tb3_bug_nav_classic')

    world_default = PathJoinSubstitution([
        tb3_pkg,
        'worlds',
        'custom_map.world'
    ])

    map_yaml_default = PathJoinSubstitution([
        tb3_pkg,
        'maps',
        'custom_map.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_default),
        DeclareLaunchArgument('map_yaml', default_value=map_yaml_default),
        DeclareLaunchArgument('start_x', default_value='-2.76177'),
        DeclareLaunchArgument('start_y', default_value='-6.63469'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='-2.37407'),
        DeclareLaunchArgument('goal_y', default_value='13.9167'),
        DeclareLaunchArgument('goal_z', default_value='0.0'),
        DeclareLaunchArgument('occupied_threshold', default_value='200'),
        DeclareLaunchArgument('allow_diagonal', default_value='true'),
        DeclareLaunchArgument('safety_radius', default_value='0.4'),
        DeclareLaunchArgument('simplify_distance', default_value='0.3'),
        DeclareLaunchArgument('lookahead', default_value='2'),
        DeclareLaunchArgument('gui', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                tb3_pkg,
                'launch',
                'tb3_custom_map.launch.py'
            ])),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'start_yaw': LaunchConfiguration('start_yaw'),
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'goal_z': LaunchConfiguration('goal_z'),
            }.items()
        ),

        TimerAction(period=4.0, actions=[OpaqueFunction(function=_launch_planner)]),
        TimerAction(period=6.0, actions=[OpaqueFunction(function=_launch_follower)]),
    ])
