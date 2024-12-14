# hero_chassis_controller

## Overview

This is a hero robot chassis controller.You can use this controller to control the hero robot.

**Keywords:** RoboMaster, ROS, ros_control

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Péter Fankhauser<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Péter Fankhauser, pfankhauser@anybotics.com**

The PACKAGE NAME package has been tested under [ROS] Indigo, Melodic and Noetic on respectively Ubuntu 14.04, 18.04 and
20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/YoujianWu/rm_description_for_task.git)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib
- roscpp
- roslint
- control_toolbox
- dynamic_reconfigure
- message_generation
- std_msgs
- realtime_tools
- nav_msgs
- tf2_ros
- geometry_msgs
- tf2_geometry_msgs
- tf2
- tf
- rm_msgs
- rm_common

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using

	cd catkin_workspace/src
	git clone git@github.com:ZiDXie/hero_chassis_controller.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

Run the simulation and controller with:

	roslaunch hero_chassis_controller hero.launch  

## Config files

* **controllers.yaml** Params of hero_chassis_controller,joint_state_controller  wheel_base,wheel_track,wheel_radius and control mode.

## Launch files

* **hero.launch:** Hero chassis  simulation and hero chassis controller.


## Nodes

### hero_chassis_controller

Control the hero chassis.

#### Subscribed Topics

* **`/cmd_vel`** ([[geometry_msgs/Twist](http://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html) ])

  Subscribe the speed commond.

#### Published Topics

* **`/power_limit_pub`** ([[std_msgs::Float64](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)])

  Publish power limit.
  
* **`/power_pub`** ([[std_msgs::Float64](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)])

  Publish power.
  
* **`/odom`** ([[nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)])

  Publish the tf change.

#### Parameters

* **`wheel_base`** (double, default: 0.4)
* **`wheel_track`**(double, default: 0.4)
* **`chassis_mode`**(bool, default: true)
* **`power_limit`**(double, default: 80.0)
* **`effort_coeff`**(double, default: 10.0)
* **`vel_coeff`**(double, default: 0.0060)
* **`power_offset`**(double, default: -8.41)
* **`accel`**
* **`p`** (double, default: 1.0)
* **`i`** (double, default: 0.0)
* **`d`** (double, default: 0.0)

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/gdut-dynamic-x/rm_template/issues).
