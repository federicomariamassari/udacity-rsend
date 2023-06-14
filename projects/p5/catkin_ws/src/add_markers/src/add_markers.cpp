// Modified from: https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
#include "add_markers/add_markers.h"

using namespace std;

double robot_x, robot_y;

void process_odom_data_callback(const nav_msgs::Odometry &msg)
{
  robot_x = msg.pose.pose.position.x;
  robot_y = msg.pose.pose.position.y;
}

bool is_target_reached(double &x1, double &y1, double &x2, double &y2)
{
  return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2)) < 0.15;  // eps = 15 cm;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber marker_sub = n.subscribe("odom", 10, process_odom_data_callback);

  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  // Marker is initially displayed at pick-up zone
  marker.pose.position.x = 3.60;
  marker.pose.position.y = 4.00;
  marker.pose.position.z = 0.08;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Marker's sides in meters
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Set marker's color to a bright cyan (#$00ffff)
  marker.color.r = 0.00f;
  marker.color.g = 1.00f;
  marker.color.b = 1.00f;
  marker.color.a = 1.0;

  // Marker will only disappear when deleted
  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1) {

    if (!ros::ok()) {
      return 0;
    }

    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  bool is_item_picked_up = false;
  int marker_rotation = 0;

  while(ros::ok()) {

    bool success = is_target_reached(robot_x, robot_y, marker.pose.position.x, marker.pose.position.y);

    if (!is_item_picked_up) {

      if (success) {

        // Simulate a pick-up
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);

        ROS_INFO("Item successfully picked up!");
        is_item_picked_up = true;
        ros::Duration(5.0).sleep();

        // Republish marker at drop-off zone
        marker.pose.position.x = -6.40;
        marker.pose.position.y = -4.50;
      }

    } else {

      if (success) {

        // Simulate a drop-off
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);

        ROS_INFO("Item successfully delivered!");
        ros::Duration(5.0).sleep();
        break;
      }
    }

    marker_pub.publish(marker);

    switch(marker_rotation) {

      case 0:
        marker_rotation = 1;
        
        // Normalized quaternion coordinates from Blender
        marker.pose.orientation.z = 0.383;
        marker.pose.orientation.w = 0.924;
        break;

      case 1:
        marker_rotation = 0;
        marker.pose.orientation.z = 0.00;
        marker.pose.orientation.w = 1.00;
        break;
    }

    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
