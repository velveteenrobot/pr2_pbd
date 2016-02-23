#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pr2_arm_control/normal_buffer.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clicked_point_with_normal_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("clicked_point", 1, cloud_cb, );

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("clicked_point_with_normal", 1);

  // Spin
  ros::spin ();
}

