from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Arguments compartits ---
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='UR robot type'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    ur_type = LaunchConfiguration('ur_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Gazebo + controllers (ur_simulation_gazebo) ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_simulation_gazebo'),
                'launch',
                'ur_sim_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- MoveIt + RViz (ur_moveit_config) ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': ur_type,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # --- LaunchDescription final ---
    return LaunchDescription([
        ur_type_arg,
        use_sim_time_arg,
        gazebo_launch,
        moveit_launch,
    ])
