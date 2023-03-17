# Project 2: Go Chase It!

Perception, decision making, actuation.

!['Go Chase It!' Animated GIF](./img/mov2.gif)

## Project Structure

To build the tree structure (Linux):
```bash
sudo apt install tree
```

Then `cd` to the target directory and run:
```bash
tree
```

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

* `catkin_ws` is the Catkin workspace, a top-level directory in which Catkin packages are both installed and modified. These are in turn directories containing a variety of resources (e.g., source code for nodes, useful scripts, configuration files);
* Included packages for this project are `ball_chaser` and `my_robot`, each containing a CMakeLists.txt file with build instructions and a package.xml file with information about the package itself.
* `my_new_world.world` uses XML file format to describe all elements with respect to the Gazebo environment.
* `.launch` files, also in XML, which allow to launch multiple nodes simultaneously.
* 


## Building the Project

Step into directory `catkin_ws` and run command:

```bash
catkin_make
```

This will create folders `build` and `devel`, the latter containing script `setup.bash`, which needs to be sourced in every new terminal window in order to run ROS-related commands.

## Running the Project

You would need multiple open terminals to run this project:

### First Terminal

```bash
roscore
```

You must have `roscore` running in order for ROS nodes to communicate [1]. This step can be bypassed if the `roslaunch` command is invoked, as the latter also calls `roscore` under the hood. Among the others, the `roscore` command will start up:

- a __ROS Master__, which manages and maintains a registry of all active nodes on a system, and allows them to locate one another and communicate via message passing (Pub/Sub); and
- a __Parameter Server__ (hosted by the ROS Master), which stores parameters and configuration values that are shared among the running nodes.

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

1. http://wiki.ros.org/roscore
