[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)

# Project 4: Map My World

## Overview

## Preliminary Configurations

To ensure RTAB-Map and related visualization package work on Ubuntu 20.04-5 (UTM QEMU 7.0) and ROS Noetic, below changes are required:

### Update VM Specs (Recommended)

Update UTM Virtual Machine specs to 32GB RAM and 8 CPU cores (to minimize crashes during map collection phase);

### Update linked version of OpenGL

Append below lines to `~/.bashrc` to override Mesa's linked versions of OpenGL (default: 2.1) and associated shader libraries (as required by RTAB-Map Viz):

```bash
export MESA_GL_VERSION_OVERRIDE=3.2
export MESA_GLSL_VERSION_OVERRIDE=150
```

### Rebuild OpenCV from Source with Patented Modules

To enable the SURF (Speeded-Up Robust Features) algorithm, rebuild OpenCV from source including patented module `xfeature2d` (now available in `opencv-contrib`) [2] [3]:

```bash
# Create custom directory to store all downloaded packages
cd /home/$whoami/ && mkdir packages && cd packages

git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Open `CMakeLists.txt` inside `opencv` folder and turn below option ON:

```cmake
OCV_OPTION(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" ON)
```

Build OpenCV from source linking extra modules [4]. By default, libraries will be installed in `/usr/local`:

```bash
cd /home/$whoami/packages/opencv
mkdir build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=/home/$whoami/packages/opencv-contrib/modules /home/$whoami/packages/opencv

# -jN tells make(1) to run N processes in parallel; remove flag if VM tends to crash, or use -j1
make -j5
sudo make install
```

### Rebuild RTAB-Map from Source

With OpenCV built from source and installed, download RTAB-Map and perform a clean build and installation [5]:

```bash
cd /home/$whoami/packages
git clone https://github.com/introlab/rtabmap.git
cd rtabmap && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/$whoami/workspace/udacity-rsend/projects/p4/catkin_ws/devel ..
make
sudo make install
```

Finally, clone RTAB-Map ROS inside `catkin_ws/src` and build the project [5]:

```bash
cd /home/$whoami/workspace/udacity-rsend/projects/p4/catkin_ws/src
git clone https://github.com/introlab/rtabmap_ros.git
catkin_make -j4
```

### Account for Meta-Package Setup

Account for new sub-package setup of RTAB-Map ROS in Udacity's default `mapping.launch` file [1]:

#### Old

```bash
<node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
<node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">
```

#### New

```bash
<node pkg="rtabmap_slam" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">
<node pkg="rtabmap_viz" type="rtabmap_viz" name="rtabmap_viz" args="-d $(find rtabmap_viz)/launch/config/rgbd_gui.ini" output="screen">
```

## Resources

1. [Migration Guide New Interface Noetic/ROS2](http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Fnoetic_and_newer.Migration_Guide_New_Interface_Noetic.2FROS2)

2. [RTAB-Map ROS Noetic Installation Guide](https://github.com/introlab/rtabmap_ros)

3. [RTAB-Map ROS Noetic Installation Wiki (Ubuntu)](https://github.com/introlab/rtabmap/wiki/Installation#ubuntu)

4. [OpenCV Extra Modules Installation](https://github.com/opencv/opencv_contrib)

5. [RTAB-Map ROS and Non-Free OpenCV](https://answers.ros.org/question/232015/problem-with-rtabmap_ros-and-nonfree-opencv/)
