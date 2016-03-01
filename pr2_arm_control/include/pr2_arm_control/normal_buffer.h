#ifndef NORMAL_BUFFER_H
#define NORMAL_BUFFER_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

namespace pr2_arm_control
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class NormalBuffer {

private:
    tf::TransformListener tf_;
    PointCloud::Ptr point_cloud_;
    ros::Publisher point_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber point_sub_;

public:
    NormalBuffer();
    void cloud_cb(const PointCloud::ConstPtr& input);
    void clicked_point_cb(const geometry_msgs::PointStampedConstPtr& clicked_point);

};

}

#endif // NORMAL_BUFFER_H
