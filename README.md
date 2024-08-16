
## Overview

This is a ROS package for controlling various functions of the UR5E which are not included in the official drivers (as far as I know).

## Requirements

This program requires a system set up with Ubuntu 20.04 and ROS Noetic. Install the required Universal Robots ROS drivers from [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) and [here](https://github.com/ros-industrial/universal_robot).

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

You can make the UR5E avoid force by letting it go in the same Cartesian direction of the force. It depends on MoveIt, so it can take some time or might not function at all at times. I don't need to use it more extensively than it is now, so I don't think I am going to update this. You can run the force avoidance by:

```bash
$ rosrun ur5e_control force_avoidance.py
```

### Force Magnitude Publisher

It is a simple function that adds and takes the square root of the /wrench/force's x, y, and z squared. You can run the force magnitude publisher by:

```bash
$ rosrun ur5e_control force_magnitude_publisher.py
```

### Storing Joint Positions and Moving the Robot to Those Positions

You can save positions based on 6 joint angles. The menu is self-explanatory. You can run move joints by:

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

### Logging Wrench Data to CSV

This feature allows you to log wrench data from the `/wrench` topic into a CSV file based on commands provided in a text file. The data includes force and torque components along with the timestamp.


1. **Specify paths in the script**:
    - Update the`output_dir` path in the script with the appropriate file path.

2. **Run the logger**:
    ```bash
    rosrun ur5e_control graphing_and_csv.py
    ```

3. **Control logging**:
    - Use GUI to start and stop logging

4. **CSV Output**:
    - When logging is stopped, the collected data is saved to a CSV file in the specified output directory.



## Dependencies

- ROS
- `geometry_msgs` package

Ensure you have the necessary dependencies installed and sourced before running the script.

---

If you need any further adjustments or have additional content to add, feel free to let me know!
