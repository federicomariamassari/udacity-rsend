[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)

# Project 4: Map My World

## Overview

This project implements simultaneous localization and mapping (SLAM) in ROS.

__Mapping__

__Localization__

__Figure 1: The Redesigned World__
![](./img/img2.png)

## Project Structure

__Figure 2: Directory Structure Tree__

```bash
.
└── catkin_ws
    └── src
        └── my_robot
            ├── CMakeLists.txt
            ├── config
            │   └── mapmyworld_config.rviz
            ├── launch
            │   ├── localization.launch
            │   ├── mapping.launch
            │   ├── robot_description.launch
            │   ├── teleop.launch
            │   └── world.launch
            ├── maps
            │   └── rtabmap.db (.tar.xz)
            ├── meshes
            │   ├── hokuyo.dae
            │   ├── my_coffee_table.dae
            │   └── my_curtain.dae
            ├── models
            │   └── ...
            ├── package.xml
            ├── urdf
            │   ├── my_robot.gazebo
            │   └── my_robot.xacro
            └── worlds
                ├── empty.world
                └── my_world.world
```

## Building the Project

RTAB-Map and associated visualization tool have [specific build requirements](p4-preliminary-config.md) that need to be met in order to run the project.

## Running the Project

To avoid continuous sourcing of `setup.bash` script, append below two lines to `~/.bashrc`. The change will affect all newly-spawned terminal windows. `$ROS_DISTRO` is automatically set to `noetic` at runtime.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/workspace/udacity-rsend/projects/p4/catkin_ws/devel/setup.bash
```

Then, open three terminal windows:

### First Terminal

```bash
roslaunch my_robot world.launch
```

### Second Terminal

```bash
roslaunch my_robot teleop.launch
```

### Third Terminal

```bash
roslaunch my_robot mapping.launch
```

## Code Logic

### RQt Graph

The RQt graph for the project appears in Figure 3.

__Figure 3: RQt Graph__
![RQt Graph](./img/img3.png)

__Figure 4: 2D Occupancy Grid__
![](./img/img4.png)

__Figure 5: 3D Point-Cloud Map__
![](./img/img5.png)

[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)
