# Turtlebot operation in Gazebo

Here will be scripts, sources and other files for performing vSLAM on Turtlebot in Gazebo environments. Now here is script `turtlebot_mover.py` which contains code for moving Turtlebot in simulated environments by direct setting its coordinates.

## Requirements

* Linux-based system with ROS installed
* ROS package `turtlebot3_gazebo` (can be found at [https://wiki.ros.org/turtlebot3_gazebo](https://wiki.ros.org/turtlebot3_gazebo))
* Python 2.7 or higher

## Turtlebot Mover Description

Turtlebot movement is performed with `TurtlebotMover` class of script `turtlebot_mover.py`.

This class has the following params:

* `name` - name of Turtlebot model in Gazebo. Default is `turtlebot3`
* `max_angular_speed` - maximum speed of Turtlebot's rotation (in rad/s). Without constraints, Turtlebot will be rotate with such angular speed. Default is 0.5
* `max_linear_speed` - maximum speed of Turtlebot's movement (in m/s). In case of moving by line, or by circle with enough big radius, Turtlebot moves with this linear speed. Default is 0.5
* `trajectory_type` - type of Turtlebot's trajectory. If this parameter is set as `line`, Turtlebot rotates until it is oriented to destination, then moves directly. If this parameter is set as `circle`, Turtlebot moves along arc of circle until it is oriented to destination, then moves directly. Default is `line`

Movement is performed with `move_to` method of the class. This method takes two float numbers - coordinates of the destination.

## Running Turtlebot Mover

To move Turtlebot in Gazebo environments, make the following steps:

1) Start ROS core

2) In other terminal, choose Turtlebot model as `waffle_pi`:

`export TURTLEBOT3_MODEL=waffle_pi`

3) Launch Gazebo with Turtlebot in some environment, for example, `turtlebot_house`:

`roslaunch turtlebot3_gazebo turtlebot3_house.launch`

4) In one more terminal, start Python and move Turtlebot using `TurtlebotMover` class:

```python
from turtlebot_mover import TurtlebotMover
mover = TurtlebotMover(max_angular_speed=0.5, max_linear_speed=0.5, trajectory_type='line')
mover.move_to(x, y)
```

If you have many Pythons (for example, system Python and Anaconda Python), you may face with some Python errors. To fix these errors, set your `PATH` system variable to satisfy ROS's python (for example, remove Anaconda from `PATH` if your ROS is with system Python)