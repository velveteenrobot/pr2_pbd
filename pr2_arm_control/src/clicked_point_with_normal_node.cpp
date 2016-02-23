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
  ros::NodeHandle nh;

  // Create cloud buffer object
  pr2_arm_control::NormalBuffer buffer;

  // subscriber for input point cloud
  ros::Subscriber cloud_sub = nh.subscribe ("/head_mount_kinect/depth_registered/points", 1, &pr2_arm_control::NormalBuffer::cloud_cb, &buffer);
  
  // subscriber for input clicked point
  ros::Subscriber point_sub = nh.subscribe ("/clicked_point", 1, &pr2_arm_control::NormalBuffer::clicked_point_cb, &buffer);

  // Spin
  ros::spin ();
}

