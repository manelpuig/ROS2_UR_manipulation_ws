## UB custom Docker-based ROS2 Humble UR5eenvironment

We have designed a University of Barcelona custom Docker-based ROS 2 Humble environment to simplify student access to ROS 2 and ensure platform-independent workflows in robotics courses.


**PC-ubuntu/linux** will work on SIM and LAB use. docker-compose.yaml is configured by default for LAB use.

- Open a terminal in `~/my_rUBot_mecanum/network_config/humble` and run:
    ````bash
    xhost +local:root            # only in case of Host Ubuntu to allow X11 for Docker 
    docker compose up
    ````
- Verify the environment variables are correctly set by checking the container startup output.
- In Host VScode you can `attach VScode`. You can also connect with container typing:
    ```bash
    docker exec -it pc_humble bash
    code .  # to open VSCode inside the container
    ```
- Clone your ws in `/root/`
- Verify in container **.bashrc** to have:
    ```bash
    source /opt/ros/humble/setup.bash
    source /root//my_rUBot_mecanum/install/setup.bash
    export QT_QPA_PLATFORM=xcb  # good default for RViz2 on many systems
    cd /root/my_rUBot_mecanum
    ```
You are ready to work inside the container and to connect to the robot hardware within ROS2 Humble on Docker!

- To stop the container, open a new terminal on Host in `~/my_rUBot_mecanum/network_config/humble` and run:
    ```bash
    docker compose down
    ```
- To see the Images and Containers:
    ```bash
    docker ps -a               # containers
    docker images              # images
    ```
- To modify the `Dockerfile`, build and push to Docker Hub, you can follow the instructions:
    ```bash
    docker build -t manelpuig/ros2-humble-ub-biorob:latest .
    docker login
    docker push manelpuig/ros2-humble-ub-biorob:latest
    ```