# Setup Instructions

These are the minimal and clean steps to install and run the UR5e simulation using ROS 2 Humble, Gazebo Classic, and MoveIt2 on Ubuntu 22.04.


Webgraphy:
- https://docs.ros.org/en/humble/p/pymoveit2/index.html

---

## 1. Create a clean workspace

```bash
mkdir -p ~/ROS2_UR_manipulation_ws/src
cd ~/ROS2_UR_manipulation_ws
```

## 2. clone the required repositories
```bash
cd ~/ROS2_UR_manipulation_ws/src
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation.git
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials/.git
```
You have to delete the .git folder:

```bash
rm -rf Universal_Robots_ROS2_Gazebo_Simulation/.git
rm -rf Universal_Robots_ROS2_Tutorials/.git
```
This will be done if you clone your forked github repository

## 3. Install `pymoveit` package

Basic Python interface for MoveIt 2 built on top of ROS 2 actions and services. (https://docs.ros.org/en/humble/p/pymoveit2/index.html
)

Instructions to install:
- Clone this repository, install dependencies and build with colcon.

    ```bash
    # Clone this repository into your favourite ROS 2 workspace
    git clone https://github.com/AndrejOrsula/pymoveit2.git
    # Install dependencies
    rosdep install -y -r -i --rosdistro ${ROS_DISTRO} --from-paths .
    # Build
    colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
    ```
- Delete some folders to sync the ws on remote github
    ```bash
    cd ~/Desktop/ROS2_UR_manipulation_ws/src/pymoveit2
    sudo rm -rf .git
    cd ~/Desktop/ROS2_UR_manipulation_ws
    git rm --cached src/pymoveit2
    ```

Verify the `.bashrc` file contains::

```bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ROS2_UR_manipulation_ws/install/setup.bash
cd ROS2_UR_manipulation_ws
```
## 4. Install the UR ROS2 environment

You will need to execute the `install_ur_moveit_gazebo_humble.sh` script to install the needed environment for Universal Robots packages and moveit2 current version.

```bash
cd ROS2_UR_manipulation_ws
chmod +x install_ur_moveit_gazebo_humble.sh
./install_ur_moveit_gazebo_humble.sh --ws ~/ROS2_UR_manipulation_ws
```
This will:
- Set the Gazebo keys 
- Install the UR packages by APT
- Install the current version of moveit if there is another older installation
- Install dependencies for UR repositories you cloned
- Build the workspace

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
