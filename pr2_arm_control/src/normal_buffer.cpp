#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <pr2_arm_control/normal_buffer.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace pr2_arm_control
{

NormalBuffer::NormalBuffer() {
    ros::NodeHandle nh;
    point_cloud_ = boost::make_shared<PointCloud>(640,480);
    // publisher for output clicked point
    point_pub_ = nh.advertise<geometry_msgs::PoseStamped>("clicked_point_normal", 1);
    // marker publisher
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    // subscriber for input point cloud
    cloud_sub_ = nh.subscribe ("/head_mount_kinect/depth_registered/points", 1, &NormalBuffer::cloud_cb, this);
    // subscriber for input clicked point
    point_sub_ = nh.subscribe ("/clicked_point", 1, &NormalBuffer::clicked_point_cb, this);
}

void NormalBuffer::cloud_cb(const PointCloud::ConstPtr& input)
{
  // save a copy of the point cloud
  *point_cloud_ = *input;
}

// Computes the normal at a clicked point
void NormalBuffer::clicked_point_cb(const geometry_msgs::PointStampedConstPtr& clicked_point)
{
  ROS_INFO_STREAM("Point clicked: \n" << *clicked_point);

  // transform clicked point from RViz fixed frame back into camera coordinates
  geometry_msgs::PointStampedPtr clicked_point_transformed(new geometry_msgs::PointStamped());
  std::string destination_frame = "/head_mount_kinect_rgb_optical_frame";
  std::string original_frame = clicked_point->header.frame_id;
  tf_.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(10.0));
  tf_.transformPoint(destination_frame, ros::Time(0), *clicked_point, original_frame, *clicked_point_transformed);
  ROS_INFO_STREAM("point before transform: \n" << *clicked_point);
  ROS_INFO_STREAM("point after transform: \n" << *clicked_point_transformed);

  // finds indices of nearest point in the point cloud
  pcl::PointXYZ search_point;
  search_point.x = clicked_point_transformed->point.x;
  search_point.y = clicked_point_transformed->point.y;
  search_point.z = clicked_point_transformed->point.z;

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 0.02;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(point_cloud_);
  kdtree.radiusSearch(search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

  // estimate local normal
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  const std::vector<int> indices;
  float nx, ny, nz, curvature;
  ne.computePointNormal(*point_cloud_, pointIdxRadiusSearch, nx, ny, nz, curvature);
  pcl::flipNormalTowardsViewpoint(search_point, 0.0, 0.0, 0.0, nx, ny, nz);

  // publish clicked point with normal
  geometry_msgs::PoseStamped point_normal_stamped;
  point_normal_stamped.header = clicked_point_transformed->header;
  point_normal_stamped.pose.position = clicked_point_transformed->point;

  tf::Vector3 axis_vector(nx,ny,nz);
  tf::Vector3 up_vector(1.0, 0.0, 0.0);
  tf::Vector3 right_vector = axis_vector.cross(up_vector);
  right_vector.normalized();
  tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
  q.normalize();
  geometry_msgs::Quaternion normal_orientation;
  tf::quaternionTFToMsg(q, normal_orientation);

  point_normal_stamped.pose.orientation = normal_orientation;

//  point_normal_stamped.point_normal.normal.x = nx;
//  point_normal_stamped.point_normal.normal.y = ny;
//  point_normal_stamped.point_normal.normal.z = nz;

  // visualize point normal
  visualization_msgs::Marker arrow_marker;
  arrow_marker.ns = "click_point_normal";
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.header = point_normal_stamped.header;
  arrow_marker.pose = point_normal_stamped.pose;
  arrow_marker.scale.x = 0.08;
  arrow_marker.scale.y = 0.02;
  arrow_marker.scale.z = 0.02;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.a = 1.0;
  ROS_INFO_STREAM("arrow_marker: \n" << arrow_marker);
  marker_pub_.publish(arrow_marker);

  // publish point normal
  point_pub_.publish(point_normal_stamped);
}

}
