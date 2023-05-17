[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)

# Project 4: Map My World

## Overview

This project implements Simultaneous Localization And Mapping (SLAM) in ROS via RTAB-Map.

__Mapping__ is the process of generating a 2D occupancy grid and a 3D point-cloud map on the fly:

Loop closure used to determine if location has been seen before. As robot travels to new areas in its environment, the map is expanded. SURF (Speeded-Up Robust Features) is used to extract visual features from a map. If enough features in an image have been detected before, then the loop is closed. 

__Mapping__ is the process of generating a 2D occupancy grid and 3D point-cloud map on the fly: the robot moves across the room collecting data from visual sensors, and from these data features are extracted and added to the robot's bag-of-words.

The robot moves across the room collecting data from visual sensors which are added to its bag-of-words. Loop closure is used to determine whether a location has been seen before.

Localization is about understanding if a location has been seen before.
To perform __Localization__, as per Project 3, requires a pre-existing map. The robot navigates the environment until it visually recognizes a landmark location existing in the database (one global loop closure), at which point the complete map is displayed.
First, need to teleport robot in an unknown location, then movement until the map is recognized. 

__Figure 1: The Redesigned World__
![](./img/img2.png)

## Project Structure

The directory structure tree for this project is outlined in Figure 2. Worth of mention are launch files `mapping.launch` and `localization.launch` performing, respectively, SLAM and pure localization, and `rtabmap.db`, the generated map database of the environment (here, compressed and split into smaller archives).

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

The RQt graph for the project appears in Figure 3. RTAB-Map acquires data from (i.e., subscribes to) both laser scan (`/scan`) and RGB-D camera (`camera/rgb/image_raw` and `camera/depth/image_raw`) and publishes to topic `/map` to generate the map. To operate the robot `/teleop_twist_keyboard` is used.

__Figure 3: RQt Graph__
![RQt Graph](./img/img3.png)

The camera is located at 10 cm (?) from the ground, so even at a large distance the robot is unable to collect images on tall surfaces (result unmapped) or top of non-transparent.. As a consequence, the 2D grid only displays obstacles related to object legs (table, chairs) - this is how it should be as robot is able to traverse these objects freely - and the 3D map has several obscure areas (no info available). Nevertheless, the resulting images are quite precise. Also because the depth camera generates good images from a maximum distance of 4 meters, so going further than that will likely result in black pictures.

__Figure 4: 2D Occupancy Grid__
![](./img/img4.png)

__Figure 5: 3D Point-Cloud Map__
![](./img/img5.png)

[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)
