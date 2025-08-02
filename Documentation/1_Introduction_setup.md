# Introduction to Robotics Manipulation

In robotics, manipulation refers to a robot's ability to interact with objects in its environment. 

A typical example is picking up an object and moving it to a new location.

The objectives of this course are:
- How to set up a MoveIt2 configuration package for a manipulator robot
- How to use MoveIt2 in ROS2 programmatically in C++
- Different types of motion planning
- How to use Perception to find object coordinates in the environment
- How to create a Pick and Place task in ROS2

Webbiography:
- https://github.com/UniversalRobots
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Documentation
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
- https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation

## 1. Set up a UR project in virtual environment for simulation

For **simulation** we will use TheConstruct interface. When working in Laboratory groups, we suggest you:
- One student plays the role of `Director`. This student makes a "Fork" of the Professor's github project.
- The `Director` accept the other students as `Collaborators`
![](./Images/01_Setup/github_collaborators.png)
- Then the `Collaborators` will make a "fork" of the `Director`'s github project.
- The `Collaborators` will be able to update the github `Director`'s project and participate on the project generation

To work on the project (during lab sessions or for homework), each student has to clone the `Director`'s github project in the `TheConstruct working environment`.
- Open your ROS2 Humble environment:  https://app.theconstructsim.com/
- Open your created ROS2_Humble Rosject project
- First time, clone your forked `Director`'s github project
  ```shell
  cd /home/user
  git clone https://github.com/director_username/ROS2_UR_manipulation_ws
  cd ROS2_UR_manipulation_ws
  colcon build
  ```
  >Successives times, in TheConstruct simulation environment, you can update the project with:
  ```shell
  git pull
  ```
- Add in .bashrc the lines:
  ````shell
  export ROS_DOMAIN_ID=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export GAZEBO_MODEL_PATH=/home/user/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
  source /opt/ros/humble/setup.bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  source /home/user/ROS2_rUBot_tutorial_ws/install/setup.bash
  source /home/user/ROS2_rUBot_mecanum_ws/install/setup.bash
  source /home/user/ROS2_UR_manipulation_ws/install/setup.bash
  #cd /home/user/ROS2_rUBot_tutorial_ws
  #cd /home/user/ROS2_rUBot_mecanum_ws
  cd /home/user/ROS2_UR_manipulation_ws
  ````
- If the compilation process returns warnings on "Deprecated setup tools", proceed with:
  ````shell
  sudo apt install python3-pip
  pip3 list | grep setuptools
  pip3 install setuptools==58.2.0
  ````
- If the compilation process returns wardings on PREFIX_PATH:
We will need to install the following packages:
````shell
unset COLCON_PREFIX_PATH
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
rm -rf build/ install/ log/
colcon build
source install/setup.bash
````
We have now our workspace ready with the gripper only used for simulation purposes.


## **3. Update and syncronize the repository project**

When working in Laboratory groups, we suggest you:

- `Before working on the project`, update the local repository with possible changes in github origin repository
  ````shell
  git pull
  ````
- You can work with your local repository for the speciffic project session
- `Once you have finished and you want to syncronize the changes` you have made and update the github origin repository, type:
  ````shell
  git add .
  git commit -m "Message"
  ````
- When you will Push them, the first time you will be asked to link the repository to your github account:
- Open a terminal in and type your credentials:
  ```shell
  git config --global user.email "xxx@alumnes.ub.edu"
  git config --global user.name "your_github_username"
  git commit -m "Message"
  git push
  ```
  > change the email and username and message
- You will have to specify your Username and Password (Personal Access Token you have generated)

To obtain the **PAT** in github follow the instructions:

  - Log in to GitHub
  - Click on your profile picture and select `settings`
  - Select `Developer Settings`
  - Select Access Personal Access Tokens: Choose Tokens (classic)
  - Click Generate new token (classic) and configure it:
    - Add a note to describe the purpose of the token, e.g., "ROS repo sync."
    - Set the expiration (e.g., 30 days, 60 days, or no expiration).
    - Under Scopes, select the permissions required:
      - For repository sync, you usually need: repo (full control of private repositories)
    - Click Generate token
  - Once the token is generated, copy it immediately. You won't be able to see it again after leaving the page.

The `Director`'s github repository has been updated!