
## Overview

This is a ROS package for controlling various functions of the UR series robots which are not included in the official drivers. Despite the name it can be used for every UR series robot.

## Requirements

This program requires a system set up with Ubuntu 20.04 and ROS Noetic. Install the required Universal Robots ROS drivers from [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) and [here](https://github.com/ros-industrial/universal_robot).
Additionally, this package includes a Docker container with all dependencies installed. 

### Docker Setup
Build the docker image by running:

```bash
$ sudo docker build -t ros-noetic-container .
```


And run the container by:
```bash
$ sudo docker run -it --rm --env="DISPLAY" --volume="/etc/group:/etc/group:ro" --volume="/etc/passwd:/etc/passwd:ro" --volume="/etc/shadow:/etc/shadow:ro" --volume="/etc/sudoers.d:/etc/sudoers.d:ro" --net host --privileged -v /home:/home -v ~/Volumes:/home/usr/ ros-noetic-container
```

Full guide for setting up Docker can be found [here](https://medium.com/@sepideh.92sh/how-docker-revolutionizes-application-development-a-comprehensive-guide-for-beginners-fc2d3e53eb31).

## Setting Up

First, start your robot drivers with:

```bash
$ roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=<robot_ip>
```

Then, start MoveIt with:

```bash
$ roslaunch ur5e_moveit_config moveit_planning_execution.launch
```

## Functions

### Force Avoidance

You can make the UR series robotavoid force by letting it go in the same Cartesian direction of the force. It depends on MoveIt, so it can take some time or might not function at all at times. I don't need to use it more extensively than it is now, so I don't think I am going to update this. You can run the force avoidance by:

```bash
$ rosrun ur5e_control force_avoidance.py
```

### Force Magnitude Publisher

It is a simple function that adds and takes the square root of the /wrench/force's x, y, and z squared. You can run the force magnitude publisher by:

```bash
$ rosrun ur5e_control force_magnitude_publisher.py
```

### Storing Joint Positions and Moving the Robot to Those Positions

You can save positions of the robot based on different options. See the menu for more information. You can run move joints by:

```bash
$ rosrun ur5e_control move_ur5e_joint.py
```

### Forward Kinematics Demo

This is a demo in development. You can move joints based on Cartesian coordinates without inverse kinematics calculations. This is made for demonstrating the possibility of controlling the robot spontaneously. It doesn't really do anything for now.

```bash
$ rosrun ur5e_control forward_kinematics.py _axis_choice:=<axis_choice> _move_frequency:=<move_frequency>
```

Axis choice can be: x, y, z, xy, xz, yz, xyz. This is done so you can test it freely. If left empty, it will default to xyz.

Move frequency is in seconds. If left empty, it will default to 1 second.

### Force to Velocity Controller

This works with same principal as the forward kinematics demo. Axis forces generate a velocity based on the applied joint force. You can run it by:

```bash
$ rosrun ur5e_control force_to_velocity_controller.py
```

### Logging Data to CSV

This feature allows you to log any data from the any topic into a CSV file based on commands provided in a text file. It is currently configured for the '/joint_states' topic. You can configure it to your preferences by editing the 'src/ur5e_control/graphing_and_csv.py' file.

You can run the logger by:

```bash
$ rosrun ur5e_control graphing_and_csv.py
```

### Wrench Logger GUI

This is a simple GUI for logging wrench data from the '/wrench' topic. You can run it by:

```bash
$ rosrun ur5e_control wrench_logger_gui.py
```

## Dependencies

- ROS
- `geometry_msgs` package

Ensure you have the necessary dependencies installed and sourced before running the script.

---

If you need any further adjustments or have additional content to add, feel free to let me know!
