
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  ROS_INFO("x: %f, y: %f", x, y);
}

int main(int argc, char* argv[])
{
  // This must be called before anything else ROS-related
  ros::init(argc, argv, "location_monitor");

  // Create a ROS node handle
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("odom", 10, OdomCallBack);

  // Don't exit the program.
  ros::spin();

  return 0;
}
