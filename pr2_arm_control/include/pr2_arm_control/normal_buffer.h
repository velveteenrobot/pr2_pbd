#ifndef NORMAL_BUFFER_H
#define NORMAL_BUFFER_H

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace pr2_arm_control
{

class NormalBuffer {
    ros::Publisher cloud_pub;
    ros::Publisher point_pub;
public:
    NormalBuffer();
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
    void clicked_point_cb (const geometry_msgs::PointStampedConstPtr& input);
};

}

#endif // NORMAL_BUFFER_H
