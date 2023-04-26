# Project 4: Preliminary Configurations

To ensure RTAB-Map and related visualization package work on Ubuntu 20.04-5 (UTM QEMU 7.0) and ROS Noetic, the following changes are required, in order.

## Checklist

1. Update virtual machine specifications (Recommended)
2. Update linked OpenGL versions
3. Rebuild OpenCV from source with patented modules
4. Rebuild RTAB-Map and RTAB-Map ROS from source
5. Account for RTAB-Map ROS meta-project setup

## Configurations

### 1. Update Virtual Machine Specifications (Recommended)

Update UTM Virtual Machine specs to 32GB RAM and 8 CPU cores (to minimize crashes during map collection phase).

### 2. Update Linked OpenGL Versions

To override Mesa's linked versions of OpenGL (default: 2.1) and associated shader libraries, add below lines to `~/.bashrc`:

```bash
export MESA_GL_VERSION_OVERRIDE=3.2
export MESA_GLSL_VERSION_OVERRIDE=150
```

### 3. Rebuild OpenCV from Source with Patented Modules

To enable SURF (Speeded-Up Robust Features), rebuild OpenCV from source including `xfeature2d` and other non-free modules, now available exclusively in `opencv-contrib` [1] [2]:

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

Build OpenCV linking extra modules [3]. By default, libraries will be installed in `/usr/local`:

```bash
cd /home/$whoami/packages/opencv
mkdir build && cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=/home/$whoami/packages/opencv-contrib/modules /home/$whoami/packages/opencv

# -jN tells make(1) to run N processes in parallel; remove flag if VM tends to crash, or use -j1
make -j5
sudo make install
```

### 4. Rebuild RTAB-Map and RTAB-Map ROS from Source

Download RTAB-Map and perform a clean build and installation [4]:

```bash
cd /home/$whoami/packages
git clone https://github.com/introlab/rtabmap.git
cd rtabmap && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/home/$whoami/workspace/udacity-rsend/projects/p4/catkin_ws/devel ..
make
sudo make install
```

Clone RTAB-Map ROS inside `catkin_ws/src` and build the project [4]:

```bash
cd /home/$whoami/workspace/udacity-rsend/projects/p4/catkin_ws/src
git clone https://github.com/introlab/rtabmap_ros.git
catkin_make -j4
```

### 5. Account for RTAB-Map ROS Meta-Project Setup

In Noetic, RTAB-Map ROS nodes have been migrated into sub-packages matching their functions [5]:

|**Old Name**| |**New Name**| |
|------------|----|------------|----|
|`rtabmap_ros`|`rtabmap`|`rtabmap_slam`|`rtabmap`|
|`rtabmap_ros`|`rtabmapviz`|`rtabmap_viz`|`rtabmap_viz`|

To account for the new meta-project setup, modify Udacity's default `mapping.launch` file:

```bash
<group ns="rtabmap">
    <node pkg="rtabmap_slam" type="rtabmap" name="rtabmap" output="screen" args="--delete_db_on_start">
    </node>
    ...
    <node pkg="rtabmap_viz" type="rtabmap_viz" name="rtabmap_viz" args="-d $(find rtabmap_viz)/launch/config/rgbd_gui.ini" output="screen">
    </node>
</group>
```

## Resources

1. https://github.com/introlab/rtabmap_ros
2. https://github.com/introlab/rtabmap/wiki/Installation#ubuntu
3. https://github.com/opencv/opencv_contrib
4. https://answers.ros.org/question/232015/problem-with-rtabmap_ros-and-nonfree-opencv/
5. http://wiki.ros.org/rtabmap_ros#rtabmap_ros.2Fnoetic_and_newer.Migration_Guide_New_Interface_Noetic.2FROS2
