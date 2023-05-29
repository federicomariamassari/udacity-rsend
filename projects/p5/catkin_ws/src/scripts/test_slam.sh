#!/bin/sh

init()
{
  # https://askubuntu.com/questions/161652/how-to-change-the-default-font-size-of-xterm
  xterm -fa "Monospace" -fs 11 -e "source ~/workspace/catkin_ws/devel/setup.bash ; $1" &
  sleep 5
}

# https://stackoverflow.com/questions/226703/how-do-i-prompt-for-yes-no-cancel-input-in-a-linux-shell-script
save_map()
{
  while true; do

    read -p "Do you wish to save the generated map (y/n)? " yn
    case $yn in
      # http://wiki.ros.org/map_server#map_saver
      [Yy]* ) init "rosrun map_server map_saver -f ~/workspace/catkin_ws/src/map/map";;
      * ) break;;
    esac

  done
}

# Launch custom Gazebo world and spawn robot model
init "roslaunch my_robot world.launch"

# Launch gmapping to perform SLAM
init "roslaunch my_robot gmapping.launch"

# Launch keyboard teleop with custom arguments
init "roslaunch my_robot teleop.launch"

# Choose whether to save gmapping-generated map (in main terminal window)
save_map
