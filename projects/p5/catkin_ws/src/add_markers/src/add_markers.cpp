// Modified from: https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

// https://answers.ros.org/question/262682/my-main-code-cannot-find-my-header-file
#include "add_markers/add_markers.h"

void publish_marker_at_location(visualization_msgs::Marker &marker, ros::Publisher &marker_pub, 
                                double position_x, double position_y, char* location_name)
{
  // Update the pose of the marker (selected dimensions)
  marker.pose.position.x = position_x;
  marker.pose.position.y = position_y;

  marker_pub.publish(marker);

  ROS_INFO("Publishing marker at %s zone", location_name);
  ros::Duration(5.0).sleep();
  ROS_INFO("Hiding marker at %s zone", location_name);
  ros::Duration(5.0).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our original shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok()) {
    visualization_msgs::Marker marker;

    // Set the frame_id and timestamp
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and a unique id for the marker.
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type
    marker.type = shape;

    // Set the marker action. Options are: ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Initialize marker's pose. This is a full 6 DoF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker (side in meters)
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;

    // Marker will only be close to a square (as it needs to be a properly normalized quaternion)
    marker.scale.z = 0.005;

    // Set marker's color to a bright tangerine (#$f08000)
    marker.color.r = 1.0000f;
    marker.color.g = 0.5020f;
    marker.color.b = 0.0000f;
    marker.color.a = 0.9;  // Alpha must be non-zero for marker to be visible

    // Marker will be automatically deleted after 5 seconds
    marker.lifetime = ros::Duration(5.0);

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1) {

      if (!ros::ok()) {
        return 0;
      }

      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    // Publish and hide markers at the desired zone
    publish_marker_at_location(marker, marker_pub, 3.60, 4.00, (char*) "pick-up");
    publish_marker_at_location(marker, marker_pub, -6.20, -4.50, (char*) "drop-off");
    break;
  }
}
