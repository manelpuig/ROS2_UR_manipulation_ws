# Motion Planning

- Running Gazebo-rviz Simulation
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur3
```

- Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur3e description_file:=ur.urdf.xacro moveit_config_package:=ur_moveit_config description_package:=ur_description moveit_config_file:=ur3e_moveit.srdf entity:=ur3e launch_rviz:=false
```
- You need to modify:
    ````shell
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/move_group.launch.py"]
        ),
    ````

Let's start by launching the robot arm UR3
````shell
ros2 launch ur_description view_ur.launch.py ur_type:=ur3
````
- launch move_group node, which is the core of MoveIt.
````shell
ros2 launch my_moveit_config move_group.launch.py
````
We  need a tool in order to interact with it. This is rviz:
````shell
ros2 launch my_moveit_config moveit_rviz.launch.py
````
