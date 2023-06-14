// Modified from: https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals

// https://answers.ros.org/question/262682/my-main-code-cannot-find-my-header-file
#include "pick_objects/pick_objects.h"

void navigate_to_target(double position_x, double position_y, double orientation_w, char* location_name,
                        MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal)
{
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = position_x;
  goal.target_pose.pose.position.y = position_y;
  goal.target_pose.pose.orientation.w = orientation_w;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending robot to %s zone", location_name);
  ac.sendGoal(goal);

  // Wait an infinite time for the result
  ac.waitForResult();

  // Check if the robot reached its goal
  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot successfully reached the %s zone", location_name);

  else
    ROS_INFO("The robot failed to reach the %s zone for some reason", location_name);

  // Wait a few seconds before the terminal window disappears
  ros::Duration(5.0).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_objects");

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {

    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Bring a glass of water from the kitchen (pick-up) to the bedroom (drop-off)
  navigate_to_target(3.60, 4.00, 1.570796, (char*) "pick-up", ac, goal);
  navigate_to_target(-6.40, -4.50, -1.570796, (char*) "drop-off", ac, goal);

  return 0;
}
