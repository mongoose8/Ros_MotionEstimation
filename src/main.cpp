#include "ros/ros.h"
#include "motionEstimation_node.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "motionEstimation_node");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
   motionEstimation_node me(nh, nh_private);
   
  // ros::MultiThreadedSpinner spinner(0);
 // spinner.spin();
      ros::spin();
   
   return 0;
}
