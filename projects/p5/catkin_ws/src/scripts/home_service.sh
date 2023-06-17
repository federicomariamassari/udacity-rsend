#!/bin/sh

init()
{
  # https://askubuntu.com/questions/161652/how-to-change-the-default-font-size-of-xterm
  xterm -fa "Monospace" -fs 11 -e "source ~/workspace/udacity-rsend/projects/p5/catkin_ws/devel/setup.bash ; $1" &
  sleep 5
}

# Launch custom Gazebo world, RViz, and spawn robot model (from Projects 2-4)
init "roslaunch my_robot world.launch"

# Launch gmapping to perform SLAM
# Created my own file based on https://github.com/rst-tu-dortmund/teb_local_planner_tutorials/issues/6
init "roslaunch my_robot amcl.launch"

# Launch add_markers node to display markers at pick-up and drop-off locations
init "rosrun add_markers add_markers"

# Launch pick_objects node to move to pick-up and drop-off locations
init "rosrun pick_objects pick_objects"
