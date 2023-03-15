# Project 2: Go Chase It!

## Setup

### First Terminal

```bash
roscore
```

### Second Terminal

```bash
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

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

`my_robot_config.rviz` inside `/home/$whoami/workspace/catkin_ws/src/my_robot`.

<table>
  <tr>
    <td align="center"><img align="center" src="./img/img2.png"/></td>
  </tr>
</table>
