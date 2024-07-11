#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "smoother.h"


ros::Publisher smooth_path_pub;
Smoother path_smooth_tool;

void globalPlannerCallback(const nav_msgs::Path::ConstPtr& msg) {
    nav_msgs::Path smoothed_path;
    smoothed_path = path_smooth_tool.smoothPath(msg);
    ROS_INFO("path soomth.....");
    smooth_path_pub.publish(smoothed_path);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "global_planner");
  ros::NodeHandle nh;

  smooth_path_pub = nh.advertise<nav_msgs::Path>("/my_global_planner_smooth", 10);
  ros::Subscriber global_planner_sub = nh.subscribe<nav_msgs::Path>("/my_global_planner", 10, globalPlannerCallback);
  ros::spin();
  return 0;
}
