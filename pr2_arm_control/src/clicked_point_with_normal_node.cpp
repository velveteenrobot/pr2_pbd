#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pr2_arm_control/normal_buffer.h>

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clicked_point_with_normal_node");

  // Create cloud buffer object
  pr2_arm_control::NormalBuffer buffer;

  // Spin
  ros::spin ();
}

