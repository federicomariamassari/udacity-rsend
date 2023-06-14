#ifndef ADD_MARKERS_H
#define ADD_MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

void process_odom_data_callback(const nav_msgs::Odometry &msg);

bool is_target_reached(double &x1, double &y1, double &x2, double &y2);

// Pass by reference to ensure same publisher publishes same marker instance (with same id) each time
// https://knowledge.udacity.com/questions/869735
void publish_marker_at_location(visualization_msgs::Marker &marker, ros::Publisher &marker_pub, 
                                double position_x, double position_y, char* location_name);

#endif /* ADD_MARKERS_H */
