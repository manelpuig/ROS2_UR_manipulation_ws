# Motion Planning

- Running Gazebo-rviz Simulation
```
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur3
```

- Example using MoveIt with simulated robot:
```
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```

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
