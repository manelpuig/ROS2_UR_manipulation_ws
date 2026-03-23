from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur5e")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true")
    launch_servo_arg = DeclareLaunchArgument("launch_servo", default_value="false")
    gazebo_gui_arg = DeclareLaunchArgument("gazebo_gui", default_value="true")

    ur_type = LaunchConfiguration("ur_type")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_simulation_gazebo"),
                "launch",
                "ur_sim_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": use_sim_time,
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": use_sim_time,
            "launch_rviz": launch_rviz,
            "launch_servo": launch_servo,
            # Important: robot_description already published by ur_sim_control
            "publish_robot_description": "false",
        }.items(),
    )

    return LaunchDescription([
        ur_type_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        launch_servo_arg,
        gazebo_gui_arg,
        gazebo_launch,
        moveit_launch,
    ])
