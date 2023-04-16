[Home](../../README.md) | Next: [Where Am I?](../p3/p3-where-am-i.md)

# Project 2: Go Chase It!

Perception, decision making, actuation.

__Figure 1: The Ball-Chasing Robot__
!['Go Chase It!' Animated GIF](./img/mov2.gif)

## Project Structure

The directory structure tree [1] for the project appears in Figure 2. `catkin_ws` is the Catkin workspace, the top-level directory where packages are managed; it includes two of them:

* `ball_chaser`, which contains the ball-chasing logic (white pixel detection and robot motion);
* `my_robot`, which holds the robot design (URDF: Unified Robot Description Format), the Gazebo world, and the launch and configuration files.

__Figure 2: Directory Structure Tree__
```bash
.
└── catkin_ws
    └── src
        ├── ball_chaser
        │   ├── CMakeLists.txt
        │   ├── launch
        │   │   └── ball_chaser.launch
        │   ├── package.xml
        │   ├── src
        │   │   ├── drive_bot.cpp
        │   │   └── process_image.cpp
        │   └── srv
        │       └── DriveToTarget.srv
        └── my_robot
            ├── CMakeLists.txt
            ├── config
            │   └── ball_chaser_config.rviz
            ├── launch
            │   ├── robot_description.launch
            │   └── world.launch
            ├── meshes
            │   └── hokuyo.dae
            ├── package.xml
            ├── urdf
            │   ├── my_robot.gazebo
            │   └── my_robot.xacro
            └── worlds
                ├── empty.world
                └── my_new_world.world
```

## Building the Project

With `catkin_ws` as current directory, run:

```bash
catkin_make
```

Among the others, this command will create folders `build` and `devel`, the latter containing the `setup.bash` script which you may need to source in each new terminal window in order to run ROS-related statements.

## Running the Project

To run the project you would need multiple open terminals; later in Project 3, all commands will be added to a single `.launch` file to minimize inconveniences.

### First Terminal

```bash
roscore
```

You must have `roscore` running in order for ROS nodes to interact. This step is recommended even though it can be bypassed if the `roslaunch` command is invoked (as the latter also calls `roscore` under the hood). Among the other tasks, `roscore` will start up:

* The __ROS Master__, which manages and maintains a registry of all active nodes on a system, and allows them to locate one another and communicate via message passing (Topics: Pub/Sub) and through services (Services: Request/Response);

* A __Parameter Server__ (hosted by the ROS Master), which stores parameter and configuration values shared among the running nodes.

### Second Terminal

```bash
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

The last command will trigger the ball-chasing logic comprising a client node that subscribes to, and analyzes, the robot's camera images, and a server node that publishes to the robot's wheels and drives the robot around if a white ball is detected. The node will be waiting for incoming data until the Gazebo world, containing our little friend, is spawned.

### Third Terminal

```bash
source devel/setup.bash
roslaunch my_robot world.launch
```

This will launch the Gazebo world [Figure 3], the robot at the origin (x=0, y=0, z=0), and the RViz (ROS Visualization) widget with custom configuration based on below line from `world.launch`:

```xml
<node name="rviz" pkg="rviz" type="rviz" respawn="false" args="-d $(find my_robot)/config/ball_chaser_config.rviz"/>
```

In RViz a camera view, the robot, and a lidar point-cloud scan will appear on the screen [Figure 4]. Also, for each camera image received the `ball_chaser` node will now start publishing velocities to the robot's wheels (check the output on the second terminal).

__Figure 3: The Gazebo World Holding the Robot__
![The Gazebo World](./img/img2.png)

__Figure 4: The Camera View and Lidar Point-Cloud Scan in RViz__
![RViz Lidar View](./img/img3.png)

### Fourth Terminal

```bash
source devel/setup.bash
rosrun rqt_image_view rqt_image_view
```

This command will spawn an additional instance of the camera view, but easier to handle than the one embedded in RViz.

## Code Logic

The RQt (ROS Qt "cute" framework) graph for the project appears in Figure 5. To reproduce the same, run below commands in an additional terminal instance, then once inside the GUI set "Nodes/Topics (all)", "Group" equal to 1 (to nest Namespace boxes once), and unhide `/tf` (Transformation Tree).

```bash
source devel/setup.bash
rosrun rqt_graph rqt_graph
```

__Figure 5: RQT Graph__
![RQt Graph](./img/img4.png)

# Resources

1. Install via `sudo apt install tree` (Linux), then step into the desired directory and run `tree`.
