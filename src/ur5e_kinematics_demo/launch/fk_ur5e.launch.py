from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Joint arguments (in radians)
    joint1 = DeclareLaunchArgument('joint1', default_value='0.0')
    joint2 = DeclareLaunchArgument('joint2', default_value='-1.57')
    joint3 = DeclareLaunchArgument('joint3', default_value='1.57')
    joint4 = DeclareLaunchArgument('joint4', default_value='0.0')
    joint5 = DeclareLaunchArgument('joint5', default_value='1.57')
    joint6 = DeclareLaunchArgument('joint6', default_value='0.0')

    fk_link = DeclareLaunchArgument('fk_link', default_value='tool0')
    group_name = DeclareLaunchArgument('group_name', default_value='ur_manipulator')

    node = Node(
        package='ur5e_kinematics_demo',
        executable='ur5e_forward_kinematics_node',
        name='ur5e_fk_node',
        output='screen',
        parameters=[{
            'joint1': LaunchConfiguration('joint1'),
            'joint2': LaunchConfiguration('joint2'),
            'joint3': LaunchConfiguration('joint3'),
            'joint4': LaunchConfiguration('joint4'),
            'joint5': LaunchConfiguration('joint5'),
            'joint6': LaunchConfiguration('joint6'),
            'fk_link': LaunchConfiguration('fk_link'),
            'group_name': LaunchConfiguration('group_name'),
        }]
    )