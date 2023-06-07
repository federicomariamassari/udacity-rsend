#ifndef ADD_MARKERS_H
#define ADD_MARKERS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Pass by reference to ensure same publisher publishes same marker instance (with same id) each time
// https://knowledge.udacity.com/questions/869735
void publish_marker_at_location(visualization_msgs::Marker &marker, ros::Publisher &marker_pub, 
                                double position_x, double position_y, char* location_name);

#endif /* ADD_MARKERS_H */