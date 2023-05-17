[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)

# Project 4: Map My World

## Overview

This project is an implementation of Simultaneous Localization And Mapping (SLAM) in ROS via RTAB-Map.



In __mapping__, 

__Mapping__ is the process of generating a 2D occupancy grid and a 3D point-cloud map on the fly: the robot moves across the room using visual sensors to collect data from which features are extracted and added to the robot's bag-of-words. Portions of the environment are progressively added to the robot's bag-of-words, and contribute to map generation. LiDAR for 2D occupancy, and 3D for depth images. Loop closure is used to determine if a location has been seen before. If an area already exists in the robot's memory (based on number of features extracted), then the robot has seen it already and images are merged to create a single, more refined view of that place.

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

## Parameter Configuration

### Mapping

<table>
    <thead>
        <tr>
            <th>Type</th>
            <th>Parameter</th>
            <th>Value</th>
            <th>Rationale</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td rowspan=1>RTAB-Map Parameters</td>
            <td><code>Rtabmap/DetectionRate</code></td>
            <td><code>1</code></td>
            <td>Rate (Hz) at which new nodes are added to map (1 Hz = 1 second).</td>
        </tr>
        <tr>
            <td rowspan=1>Memory</td>
            <td><code>Mem/NotLinkedNodesKept</code></td>
            <td><code>false</code></td>
            <td>Avoid registering new images if robot is not moving.</td>
        </tr>
        <tr>
            <td rowspan=2>Keypoint Memory</td>
            <td><code>Kp/MaxFeatures</code></td>
            <td><code>400</code></td>
            <td>Maximum size of the bag-of-words.</td>
        </tr>
        <tr>
            <td><code>Kp/DetectorStrategy</code></td>
            <td><code>0</code></td>
            <td>Using SURF as loop closure detection strategy.</td>
        </tr>
        <tr>
            <td rowspan=1>Keypoint Descriptors and Detectors</td>
            <td><code>SURF/HessianThreshold</code></td>
            <td><code>100</code></td>
            <td>Used to extract more or less SURF features.</td>
        </tr>
        <tr>
            <td rowspan=1>RGBD-SLAM</td>
            <td><code>RGBD/LinearUpdate</code></td>
            <td><code>10</code></td>
            <td>Slightly increase linear displacement for map update (to 10 cm) to reduce computational burden.</td>
        </tr>
        <tr>
            <td rowspan=3>Odometry</td>
            <td><code>Odom/Holonomic</code></td>
            <td><code>false</code></td>
            <td>To account for non-holonomic, skid-steer robot setup: y = x*tan(yaw).</td>
        </tr>
        <tr>
            <td><code>Odom/FilteringStrategy</code></td>
            <td><code>2</code></td>
            <td>Use particle filtering.</td>
        </tr>
        <tr>
            <td><code>Odom/ParticleSize</code></td>
            <td><code>120</code></td>
            <td>Set number of particles to 120 (which worked well for Project 3 "Where Am I?").</td>
        </tr>
        <tr>
            <td rowspan=2>Common Registration Parameters</td>
            <td><code>Reg/Strategy</code></td>
            <td><code>0</code></td>
            <td>Use visual-only loop closure constraint (disregard scans).</td>
        </tr>
        <tr>
            <td><code>Reg/Force3DoF</code></td>
            <td><code>true</code></td>
            <td>2D SLAM: only consider x, y, and yaw (disregard z, roll, and pitch).</td>
        </tr>
    </tbody>
</table>

### Localization

## Code Logic

### RQt Graph

The RQt graph for the project appears in Figure 3. RTAB-Map acquires data from (i.e., subscribes to) both laser scan (`/scan`) and RGB-D camera (`camera/rgb/image_raw` and `camera/depth/image_raw`) and publishes to topic `/map` to generate the map. To operate the robot `/teleop_twist_keyboard` is used.

__Figure 3: RQt Graph__
![RQt Graph](./img/img3.png)

### Mapping

The RGB-D camera is at 10 centimeters from the ground, 

The camera is located at 10 cm (?) from the ground, so even at a large distance the robot is unable to collect images on tall surfaces (which then result unmapped) or top of non-transparent.. As a consequence, the 2D grid only displays obstacles related to object legs (table, chairs) - this is how it should be as robot is able to traverse these objects freely - and the 3D map has several obscure areas (no info available). Nevertheless, the resulting images are quite precise. Also because the depth camera generates good images from a maximum distance of 4 meters, so going further than that will likely result in black pictures.

__Figure 4: 2D Occupancy Grid__
![](./img/img4.png)

__Figure 5: 3D Point-Cloud Map__
![](./img/img5.png)

## Resources

1. https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h

[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)
