#include "ros/ros.h"
#include "std_msgs/String.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Hello, I am a student");
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

 
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("testpackage/advertiser", 1000, chatterCallback);

  ros::spin();

  return 0;
}