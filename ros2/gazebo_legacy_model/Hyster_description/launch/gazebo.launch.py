from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('Hyster_description')

    # The implement plugin lives in hyster_gazebo_plugins. Resolving it here makes
    # the launch fail right away if this shell was sourced before that package
    # was built, instead of spawning a model whose mast and clamps hang loose.
    plugin_dir = os.path.join(get_package_prefix('hyster_gazebo_plugins'), 'lib')
    plugin_path = SetEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.pathsep.join(filter(None, [plugin_dir, os.environ.get('GAZEBO_PLUGIN_PATH')])))

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true')

    xacro_file = os.path.join(share_dir, 'urdf', 'Hyster.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': True}
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'Hyster',
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        plugin_path,
        gui_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])
