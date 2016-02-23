#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

#include <pr2_arm_control/normal_buffer.h>

namespace pr2_arm_control
{

NormalBuffer::NormalBuffer() {
    ros::NodeHandle nh;
    // publisher for output point cloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2> ("copied_point_cloud", 1);
    // publisher for output clicked point
    point_pub = nh.advertise<geometry_msgs::PointStamped> ("copied_clicked_point", 1);
}

void NormalBuffer::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  cloud_pub.publish (output);
}

void NormalBuffer::clicked_point_cb(const geometry_msgs::PointStampedConstPtr& input)
{
  geometry_msgs::PointStampedPtr output(new geometry_msgs::PointStamped());
  *output = *input;
  ROS_INFO_STREAM("Point clicked: \n" << *output);
  point_pub.publish(output);
}

}
