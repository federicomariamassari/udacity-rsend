[Home](../../README.md) | Previous: [Map My World](../p4/p4-map-my-world.md)

# Project 5: Home Service Robot

## Overview

This capstone project combines SLAM, pure localization, navigation, and C++ programming of ROS nodes to deploy an autonomous robot that can fetch and deliver items in a custom Gazebo environment. The project builds on the experience gained from previous assignments, in particular:

- __SLAM__: The mapping task was similar to that of [Project 4](../p4/p4-map-my-world.md) (Map My World) but less computationally-intensive, since producing a 3D point-cloud view (which often made my VM crash with RTAB-Map) was not required this time. Hence, I could finally place the robot in my "real" home — no need to create an ad hoc smaller room to accommodate machine restrictions [Figure 1].

- __Pure Localization and Navigation__: The parameter calibration for these tasks was similar to, but much more involved than, that required for [Project 3](../p3/p3-where-am-i.md) (Where Am I?). Keeping the structure of the original launch and configuration files from the Turtlebot packages suggested by Udacity meant, for example, dealing with a considerably more complex ROS navigation stack.

- __Programming ROS Nodes in C++__: This final part of the assignment felt very much like doing [Project 2](../p2/p2-go-chase-it.md) (Go Chase It!) once again, with the logic of `pick_objects` and `add_markers` (the harder one) being very close to that of `drive_bot` and `process_image` respectively.

__Figure 1: My "Real" Virtual Home__
![My Virtual Home](./img/img2.png)

## Challenges Faced

As opposed to previous projects, in which the biggest hardship was to make them work locally on Ubuntu 20.04 and ROS Noetic (building RTAB-Map with SURF in Project 4 [was really painful](../p4/p4-preliminary-config.md)), this one did not pose significant challenges. Some small hurdles:

### Getting Started

I was initially disappointed to find out the capstone project relied on Turtlebot and not on the robot I built and worked with since Project 2. Turtlebot and its variant Turtlebot 3 are difficult to build in ROS Noetic, for which no compatible version of libraries such as `turtlebot_rviz_launchers` exists. Failed attempts at tackling the assignment included:

1. Trying to resolve dependencies locally by installing Noetic-compatible packages (whenever possible) or iteratively cloning the suggested repositories at each failed build, hoping the project will eventually compile successfully;

2. Scrapping the idea of running the Turtlebot project locally, and only relying on the Udacity workspace.

Finally, thanks to online posts, I realized it was feasible and actually encouraged to submit the project using my own world and robot [1] [2].

### RViz Configuration

Faithful reproduction in my custom environment of all the RViz displays that come with the Turtlebot navigation stack was not an easy feat, and it came from observing and mimicking the behaviour of the same components in the Udacity workspace setup. In the end, only "Bumper Hit" was left out because it relies on the Turtlebot-specific `/mobile_base` topic.

__Figure 3: RQt Graph__
![RQt Graph](./img/img3.png)

## Resources

1. https://knowledge.udacity.com/questions/903802
2. https://github.com/rst-tu-dortmund/teb_local_planner_tutorials/issues/6
3. http://wiki.ros.org/base_local_planner?distro=noetic

[Home](../../README.md) | Previous: [Map My World](../p4/p4-map-my-world.md)
