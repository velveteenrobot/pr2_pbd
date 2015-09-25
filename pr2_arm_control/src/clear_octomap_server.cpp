#include "ros/ros.h"
#include <moveit/move_group/clear_octomap_service_capability.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "clear_octomap_server");
  ros::NodeHandle n;

  //move_group::ClearOctomapService service;
  //service.initialize(); 

  ros::ServiceClient clear_octomap_service_client_ = nh.serviceClient<std_srvs::Empty>("/clear_octomap")
  std_srvs::Empty srv;
  clear_octomap_service_client_.call(srv);
  ros::spin();

  return 0;
}
