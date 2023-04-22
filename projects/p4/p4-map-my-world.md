[Home](../../README.md) | Previous: [Where Am I?](../p3/p3-where-am-i.md)

# Project 4: Map My World

## Overview

## Preliminary Configurations

To ensure RTAB-Map and related visualization package work on Ubuntu 20.04-5 (UTM QEMU 7.0) and ROS Noetic, below changes are required:

* Update UTM Virtual Machine specs to 32GB RAM and 8 cores (to minimize crashes during map collection phase);
* Account for new sub-package setup of RTAB-Map ROS in Udacity's default `mapping.launch` file [1]:

|__Old__|__New__|
|-------|-------|
|```bash
<node pkg="rtabmap_ros" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">```
|```bash
<node pkg="rtabmap_slam" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">```|
|```bash
<node pkg="rtabmap_ros" type="rtabmapviz" name="rtabmapviz" args="-d $(find rtabmap_ros)/launch/config/rgbd_gui.ini" output="screen">```
|```bash
<node pkg="rtabmap_viz" type="rtabmap_viz" name="rtabmap_viz" args="-d $(find rtabmap_viz)/launch/config/rgbd_gui.ini" output="screen">```|

```bash
<group ns="rtabmap">
    <node pkg="rtabmap_slam" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">
    </node>
    ...
    <node pkg="rtabmap_viz" type="rtabmap_viz" name="rtabmap_viz" args="-d $(find rtabmap_viz)/launch/config/rgbd_gui.ini" output="screen">
    </node>
</group>
```

* Override Mesa's linked versions of OpenGL (default: 2.1) and associated shader libraries by appending below lines to `~/.bashrc`:

```bash
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=150
```

## Resources

1. [Migration Guide New Interface Noetic/ROS2](http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Fnoetic_and_newer.Migration_Guide_New_Interface_Noetic.2FROS2)
