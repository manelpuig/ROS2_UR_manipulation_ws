# Setup custom UR repository

Important webgraphy:
- Youtube tutorial channel:https://www.youtube.com/@learn-robotics-with-ros
        - Gazebo: https://www.youtube.com/watch?v=qnuwTOB4DKw
        - moveit2: https://www.youtube.com/watch?v=9xbw4IDcsAU&list=PLJOHOcnvyOr31ZYoB4JDpUWWzRfpJnfqn
        - github link: https://github.com/LearnRoboticsWROS?tab=repositories
            - https://github.com/LearnRoboticsWROS/ur_yt_sim
            - https://github.com/LearnRoboticsWROS/ur5_sim_test/tree/main

You have to prepare the repository with:
````shell
sudo mkdir -p /usr/share/keyrings
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo rm /etc/apt/sources.list.d/ros2-latest.list
sudo apt update
sudo apt install dialog
````

## 1. Install official packages (binary)
Universal Robots + control stack
````shell
# UR driver & description (meta-packages provide driver and URDFs)
sudo apt install -y ros-humble-ur ros-humble-ur-description

# ros2_control & controllers
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Tools
sudo apt install -y ros-humble-rviz2 ros-humble-xacro
````
Gazebo Classic (for Humble)
````shell
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
mv Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo my_ur_simulation_gazebo
rm -rf Universal_Robots_ROS2_Gazebo_Simulation
````
MoveIt2 (for planning)
````shell
sudo apt install -y ros-humble-moveit
````

>With the above binary installs, you do not need to clone any Universal Robots repository (https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble).

## 2. Create your workspace and packages

Create custom packages on src folder:
````shell
cd ~/ROS2_UR_manipulation_ws/src

# Your bringup package (Python, for launchers/configs)
ros2 pkg create my_ur_bringup --build-type ament_python \
  --dependencies rclpy launch launch_ros xacro robot_state_publisher controller_manager

# Your MoveIt 2 config will be created with the MoveIt Setup Assistant (next section)
````

Build once (even if empty) to check the workspace is healthy:
````shell
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
````
If you have warnings about "test_require", you can avoid them with:
- In `setup.py` comment the line `tests_require=['pytest'],`
- In `package.xml` add line `<test_depend>pytest</test_depend>`
- Recompile with:
    ````shell
    cd ~/ROS2_UR_manipulation_ws
    rm -rf build/ install/ log/
    colcon build --symlink-install
    source install/setup.bash
    ````

Once the package compiles properly, add the needed folders: `launch`, `config`, `rviz`, `meshes` and `urdf`

You have to add these folders to `setup.py` as ususal

### Custom UR5e definition

We want to use the official UR repository ([UR_Gazeo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)), we will have to:
- Add dependencies on `package.xml`
- To customize the UR5e model with some sensors (i.e. camera) I add a `ur5e_custom.xacro` to urdf folder with:
    - same structure as the original one in ur_description
    - some modifications adding sensors (camera, gripper, etc)
- Add in `config` folder: 
    - `ur_controllers.yaml` (from official ur_simulation_gazebo/config folder)
    - `initial_positions.yaml` (from official Universal_Robots_ROS2_Description/config folder)
- Add a `ur5e_custom_sim_bringup.launch.py` launch file:
    - same structure as the original ones
    - add some parameters in function of the used model
- launch the ur5e custom robot in rviz and gazebo:
    ````shell
    ros2 launch my_ur_bringup spawn_ur5_camera.launch.py
    ````
- Launch the official simple models:
    ````shell
    ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
    ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
    ````

for gripper you need information from: https://github.com/KevinGalassi/Robotiq-2f-85/tree/main

## 3. Generate your MoveIt2 configuration

Create my_ur5e_moveit_config with MoveIt Setup Assistant (MSA):
````shell
ros2 run moveit_setup_assistant moveit_setup_assistant
````

- Import URDF from the ur_description package (UR5e).

- Define your planning groups, end effector, kinematics, joint limits, collision settings, etc.

- Export the configuration into:
`~/ROS2_UR_manipulation_ws/src/my_ur5e_moveit_config`

Build again:
````shell
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
source install/setup.bash
````
## 4. Repository layout (recommended)
````yaml
ROS2_UR_manipulation_ws/
└─ src/
   ├─ my_ur5e_bringup/
   │  ├─ package.xml
   │  ├─ setup.py
   │  ├─ my_ur5e_bringup/
   │  │  ├─ __init__.py
   │  │  ├─ launch/
   │  │  │  ├─ bringup_real.launch.py
   │  │  │  ├─ bringup_ursim.launch.py
   │  │  │  └─ bringup_gazebo.launch.py
   │  │  ├─ config/
   │  │  │  ├─ controllers_real.yaml
   │  │  │  └─ controllers_gazebo.yaml
   │  │  └─ rviz/
   │  │     └─ view.rviz   (optional)
   └─ my_ur5e_moveit_config/
      └─ ... (generated by MoveIt Setup Assistant)
````

## 5. Minimal config files (you can refine later)

my_ur5e_bringup/config/controllers_real.yaml (for real robot or URSim through the official driver)
````yaml
controller_manager:
  ros__parameters:
    update_rate: 125

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    scaled_joint_trajectory_controller:
      type: ur_controllers/ScaledJointTrajectoryController

scaled_joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.1
````

my_ur5e_bringup/config/controllers_gazebo.yaml (for Gazebo Classic)
````yaml
controller_manager:
  ros__parameters:
    update_rate: 250
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]
````

## 6. Launch files
- Real robot — bringup_real.launch.py
````python
# my_ur5e_bringup/launch/bringup_real.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    driver_pkg = get_package_share_directory('ur_robot_driver')
    driver_launch = os.path.join(driver_pkg, 'launch', 'ur_control.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', description='IP of the real UR controller'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'launch_rviz': launch_rviz
            }.items()
        )
    ])
````

Run:
````bash
ros2 launch my_ur5e_bringup bringup_real.launch.py robot_ip:=<ROBOT_CTRL_IP>
````
- URSim — bringup_ursim.launch.py
````python
# my_ur5e_bringup/launch/bringup_ursim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    ursim_ip = LaunchConfiguration('ursim_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    driver_pkg = get_package_share_directory('ur_robot_driver')
    driver_launch = os.path.join(driver_pkg, 'launch', 'ur_control.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('ursim_ip', description='IP of the URSim VM'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': ursim_ip,   # driver expects 'robot_ip'
                'launch_rviz': launch_rviz
            }.items()
        )
    ])
````

Run:
````bash
ros2 launch my_ur5e_bringup bringup_ursim.launch.py ursim_ip:=<URSIM_IP>
````
- Gazebo Classic — bringup_gazebo.launch.py
````python
# my_ur5e_bringup/launch/bringup_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    world = LaunchConfiguration('world')

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    ur_desc_pkg = get_package_share_directory('ur_description')

    # Use the official UR xacro + sim flag for Gazebo Classic
    xacro_file = os.path.join(ur_desc_pkg, 'urdf', 'ur.urdf.xacro')
    robot_description = Command([
        'xacro ', xacro_file,
        ' ur_type:=', ur_type,
        ' sim_gazebo:=true',
        ' prefix:=', ''   # set a prefix if spawning multiple robots
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'ur5e'],
        output='screen'
    )

    # Load controllers inside Gazebo (classic)
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('world', default_value=''),
        gazebo, rsp, spawn, jsb, arm_ctrl
    ])
````

Run:
````bash
ros2 launch my_ur5e_bringup bringup_gazebo.launch.py
````

>Because you installed Gazebo Classic support via apt, you don’t need to clone any UR Gazebo repo for this minimal setup.

## 7. Build & run (every time you change code)
````bash
cd ~/ROS2_UR_manipulation_ws
colcon build --symlink-install
source install/setup.bash
````
Choose one:
````bash
ros2 launch my_ur5e_bringup bringup_real.launch.py  robot_ip:=<ROBOT_CTRL_IP>
ros2 launch my_ur5e_bringup bringup_ursim.launch.py ursim_ip:=<URSIM_IP>
ros2 launch my_ur5e_bringup bringup_gazebo.launch.py
````