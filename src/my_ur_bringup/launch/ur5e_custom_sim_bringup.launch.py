# my_ur_bringup/launch/ur5e_custom_sim_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type   = LaunchConfiguration('ur_type')
    gui       = LaunchConfiguration('gazebo_gui')
    use_rviz  = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type',        default_value='ur5e'),
        DeclareLaunchArgument('gazebo_gui',     default_value='true'),
        DeclareLaunchArgument('launch_rviz',    default_value='true'),

        # Si el teu Xacro custom ja apunta a controladors/posicions,
        # no cal passar-los aquí. Si vols sobreescriure'ls, afegeix els args corresponents.

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ur_simulation_gazebo'),
                '/launch/ur_sim_control.launch.py'
            ]),
            launch_arguments={
                'ur_type': ur_type,
                'gazebo_gui': gui,
                'launch_rviz': use_rviz,
                # Si vols forçar que el launch oficial faci servir el teu Xacro i YAMLs:
                'description_package': 'my_ur_bringup',
                'description_file':    'ur5e_custom.xacro',
                'runtime_config_package': 'my_ur_bringup',  # o my_ur_simulation_gazebo
                'controllers_file':       'ur_controllers.yaml',
                'initial_positions_file': 'initial_positions.yaml',
                # Altres paràmetres disponibles:
                # 'safety_limits': 'true', 'safety_pos_margin': '0.15', 'safety_k_position': '20',
                # 'start_joint_controller': 'true',
                # 'initial_joint_controller': 'joint_trajectory_controller',
            }.items()
        )
    ])

