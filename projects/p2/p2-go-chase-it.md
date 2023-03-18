# Project 2: Go Chase It!



For this project

In this second project I programmed a robot chasing a white ball using ROS.

Perception, decision making, actuation.

!['Go Chase It!' Animated GIF](./img/mov2.gif)

## Project Structure

Below is the directory structure tree [1] for the project:

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
            ├── launch
            │   ├── robot_description.launch
            │   └── world.launch
            ├── meshes
            │   └── hokuyo.dae
            ├── my_robot_config.rviz
            ├── package.xml
            ├── urdf
            │   ├── my_robot.gazebo
            │   └── my_robot.xacro
            └── worlds
                ├── empty.world
                └── my_new_world.world
```

`catkin_ws` is the Catkin workspace, the top-level directory where packages are managed. This project includes two packages: `ball_chaser`, containing  the ball-chasing logic (white pixel detection and robot motion) and `my_robot`, holding the robot design (URDF: Unified Robot Description Format), the Gazebo world, as well as launch and configuration files.

## Building the Project

Inside directory `catkin_ws` run command:

```bash
catkin_make
```

Among the others, this will create folders `build` and `devel` - the latter containing `setup.bash`, which you may need to source in each new terminal window in order to run ROS-related commands.

## Running the Project

You would need multiple open terminals to run this project:

### First Terminal

```bash
roscore
```

You must have `roscore` running in order for ROS nodes to interact [2]. This step is recommended even though it can be bypassed if the `roslaunch` command is invoked (as the latter also calls `roscore` under the hood). Among the others, `roscore` will start up:

- The __ROS Master__, which manages and maintains a registry of all active nodes on a system, and allows them to locate one another and communicate via message passing (Topics: Pub/Sub) and through services (Services: request/response);

- A __Parameter Server__ (hosted by the ROS Master), which stores parameter and configuration values shared among the running nodes.

### Second Terminal

```bash
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

These commands will launch the "ball chaser" plugin, 

### Third Terminal

```bash
source devel/setup.bash
roslaunch my_robot world.launch
```

### Fourth Terminal

```bash
source devel/setup.bash
rosrun rqt_image_view rqt_image_view
```

![My World](./img/img2.png)

`my_robot_config.rviz` inside `/home/$whoami/workspace/catkin_ws/src/my_robot`.

![RViz Lidar View](./img/img3.png)

# Resources

1. Install via `sudo apt install tree` (Linux), then step into the desired directory and run `tree`.
2. http://wiki.ros.org/roscore
