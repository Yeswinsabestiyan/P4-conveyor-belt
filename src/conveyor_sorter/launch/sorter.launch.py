from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    belt_speed_arg = DeclareLaunchArgument(
        'belt_speed', default_value='0.15',
        description='Conveyor belt speed (m/s)'
    )
    spawn_rate_arg = DeclareLaunchArgument(
        'spawn_rate', default_value='0.5',
        description='Mean box spawn rate (boxes/second)'
    )
    red_prob_arg = DeclareLaunchArgument(
        'red_prob', default_value='0.35',
        description='Probability of red box'
    )
    blue_prob_arg = DeclareLaunchArgument(
        'blue_prob', default_value='0.35',
        description='Probability of blue box'
    )
    black_prob_arg = DeclareLaunchArgument(
        'black_prob', default_value='0.30',
        description='Probability of black box'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Launch RViz2'
    )
    software_render_arg = DeclareLaunchArgument(
        'software_render', default_value='false',
        description='Set LIBGL_ALWAYS_SOFTWARE=1 (use if RViz shows no solid geometry, e.g. WSL)'
    )
    set_sw_render = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1',
        condition=IfCondition(LaunchConfiguration('software_render'))
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('conveyor_sorter'),
            'config',
            'sorter_params.yaml'
        ]),
        description='Path to parameters YAML file'
    )

    # Get package share directory
    pkg_share = FindPackageShare('conveyor_sorter')

    # Robot description (xacro) - wrap in ParameterValue
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_share, 'urdf', 'main.urdf.xacro'])
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[robot_description],
    )

    # Joint State Publisher (aggregates joint states from arm controllers)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'source_list': ['/arm1/joint_states', '/arm2/joint_states'],
            'rate': 50,
        }],
    )

    # Box Spawner
    box_spawner = Node(
        package='conveyor_sorter',
        executable='box_spawner',
        name='box_spawner',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'belt_speed': LaunchConfiguration('belt_speed'),
                'spawn_rate': LaunchConfiguration('spawn_rate'),
                'red_prob': LaunchConfiguration('red_prob'),
                'blue_prob': LaunchConfiguration('blue_prob'),
                'black_prob': LaunchConfiguration('black_prob'),
            }
        ],
    )

    # Conveyor
    conveyor = Node(
        package='conveyor_sorter',
        executable='conveyor',
        name='conveyor',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {'belt_speed': LaunchConfiguration('belt_speed')},
        ],
    )

    # Color Sensor
    color_sensor = Node(
        package='conveyor_sorter',
        executable='color_sensor',
        name='color_sensor',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    # Arm Controller 1 (Red picker)
    arm1 = Node(
        package='conveyor_sorter',
        executable='arm_controller',
        name='arm1',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'arm_name': 'arm1',
                'target_color': 'red',
            }
        ],
    )

    # Arm Controller 2 (Blue picker)
    arm2 = Node(
        package='conveyor_sorter',
        executable='arm_controller',
        name='arm2',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'arm_name': 'arm2',
                'target_color': 'blue',
            }
        ],
    )

    # Stats Display
    stats_display = Node(
        package='conveyor_sorter',
        executable='stats_display',
        name='stats_display',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    # RViz
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'sorter.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        belt_speed_arg,
        spawn_rate_arg,
        red_prob_arg,
        blue_prob_arg,
        black_prob_arg,
        use_rviz_arg,
        software_render_arg,
        params_file_arg,
        set_sw_render,
        robot_state_publisher,
        joint_state_publisher,
        box_spawner,
        conveyor,
        color_sensor,
        arm1,
        arm2,
        stats_display,
        rviz,
    ])