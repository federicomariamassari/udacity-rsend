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

To enable the SURF (Speeded-Up Robust Features) algorithm, rebuild OpenCV from source including patented module `xfeature2d` (now available in `opencv-contrib`) [2] [3].

```bash
# Create custom directory to store all downloaded packages
cd /home/$whoami/ && mkdir packages

git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Open `CMakeLists.txt` inside `opencv` folder and turn on below option:

```cmake
OCV_OPTION(OPENCV_ENABLE_NONFREE "Enable non-free algorithms" ON)
```


### Account for Meta-Package Setup

* Account for new sub-package setup of RTAB-Map ROS in Udacity's default `mapping.launch` file [1]:

```bash
<group ns="rtabmap">
    <node pkg="rtabmap_slam" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">
    </node>
    ...
    <node pkg="rtabmap_viz" type="rtabmap_viz" name="rtabmap_viz" args="-d $(find rtabmap_viz)/launch/config/rgbd_gui.ini" output="screen">
    </node>
</group>
```

* To enable SURF (Speeded-Up Robust Features), rebuild OpenCV from source linking to patented module `xfeatures2d` which is available in  `opencv-contrib` [2] [3]. We suppose packages 

```bash
cd /home/$whoami/workspace
```


## Resources

1. [Migration Guide New Interface Noetic/ROS2](http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Fnoetic_and_newer.Migration_Guide_New_Interface_Noetic.2FROS2)

2. [RTAB-Map ROS Noetic Installation Guide](https://github.com/introlab/rtabmap_ros)

3. [RTAB-Map ROS Noetic Installation Wiki (Ubuntu)](https://github.com/introlab/rtabmap/wiki/Installation#ubuntu)