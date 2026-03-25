# Setup Instructions

These are the minimal and clean steps to install and run the UR5e simulation using ROS 2 Humble, Gazebo Classic, and MoveIt2 on Ubuntu 22.04.


Webgraphy:
- https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html
- https://docs.ros.org/en/humble/p/pymoveit2/index.html

---

## 1. Install the UR ROS2 environment

You can use:
- Ubuntu22 PC 
- Docker container

### Ubuntu PC

We have prepared an installation file `setup_ur_env.sh`:
````shell
cd Ros2_UR_manipulation_ws
./setup_ur_env.sh
````

### Docker container

We have prepared an image in Docker Hub: https://hub.docker.com/r/manelpuig/ros2-humble-ub-ur5e/tags

## 2. Create a clean workspace

A Clean workspace is created with all needed packages to perform Gazebo simulation and Real UR5e robot execution.
- Create a ws
    ```bash
    mkdir -p ~/ROS2_UR_manipulation_ws/src
    cd ~/ROS2_UR_manipulation_ws
    ```

- Clone the required repositories for Gazebo simulation:
    ```bash
    cd ~/ROS2_UR_manipulation_ws/src
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/.git
    ```
- You have to delete the .git folder:
    ```bash
    rm -rf Universal_Robots_ROS2_Gazebo_Simulation/.git
    rm -rf Universal_Robots_ROS2_Tutorials/.git
    ```

- Verify the `.bashrc` file contains::

    ```bash
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source ~/ROS2_UR_manipulation_ws/install/setup.bash
    cd ROS2_UR_manipulation_ws
    ```

## 5. Launch the UR5e bringup

Using Gazebo for simulation:
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e use_sim_time:=true
```
Or launching the UR5e ROS 2 driver (fake hardware):
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=1.2.3.4 use_fake_hardware:=true launch_rviz:=false
````
> the parameter robot_ip is mandatory but ignored if use_fake_hardware:=true 
---

## 6. Launch MoveIt2 for UR5e

Using Gazebo for simulation:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_sim_time:=true
```
---

This setup provides a stable UR5e simulation environment with ROS 2 Humble and Gazebo Classic.
