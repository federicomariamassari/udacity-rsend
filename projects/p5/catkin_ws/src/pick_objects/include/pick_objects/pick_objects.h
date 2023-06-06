#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void navigate_to_target(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal, 
                        double position_x, double position_y, double orientation_w, char* location_name);