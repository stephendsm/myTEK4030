#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hallo_world");
  ros::NodeHandle nh;

  ROS_INFO("Hallo world!");

  return 0;
}
