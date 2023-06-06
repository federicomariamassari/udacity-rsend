#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO_STREAM("Driving the robot in the direction specified");
  
  // Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  // Call the service and pass the requested velocities
  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service command_robot");
  }
}

/* This callback function continuously executes and reads the image data
   
   References:
     [1] - http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html (variable definition)
     [2] - https://knowledge.udacity.com/questions/717345 (overall code logic)
     [3] - https://knowledge.udacity.com/questions/801031 (stop request outside for loop)
 */
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;  // byte value corresponding to white (#%11111111)
  bool is_ball_detected = false;
  
  // Define the rightmost boundary of each screen section (right_bound == img.width)
  int left_bound = 1/3. * img.width;
  int mid_bound = 2 * left_bound;
  
  // Loop through each pixel in the image and check if there's a bright white one.
  // Then, identify if this pixel falls in the left, mid, or right side of the image.
  // Depending on the white ball position, call the drive_bot function and pass velocities to it.
  // Request a stop when there's no white ball seen by the camera.

  // Loop through the img.data in pixel (i.e., 3 RGB bytes) increments
  // img.data is a matrix of bytes with img.step rows and img.height columns
  for (int i=0; i < img.step*img.height; i+=3) {
    
    int red = img.data[i];
    int green = img.data[i+1];
    int blue = img.data[i+2];
    
    // If RGB == {255, 255, 255} we found a white pixel
    if (red == white_pixel && green == white_pixel && blue == white_pixel) {

      is_ball_detected = true;
      int screen_portion = (i % img.step) / 3;

      // Chase the white pixel
      if (screen_portion < left_bound) {
        // Drive left until the ball is in sight in the middle portion of the screen
        drive_robot(0.5, 0.5);
      
      } else if (screen_portion < mid_bound) {
        // Drive forward towards the ball
        drive_robot(0.5, 0.0);
        
      } else {
        // Drive right until the ball is in sight in the middle portion of the screen
        drive_robot(0.5, -0.5);
      }
      break;
    }
  }
  
  if (!is_ball_detected) {
    // Bring the robot to a halt if white pixel is not detected (any longer)
    drive_robot(0.0, 0.0);
  }
}

int main(int argc, char** argv)
{
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;
  
  // Define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  
  // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
  
  // Handle ROS communication events
  ros::spin();
  
  return 0;
}
