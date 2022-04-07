#ifndef HELPERS_H
#define HELPERS_H

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

template <typename PointT>
bool publish_cloud(const ros::Publisher &pub, const typename pcl::PointCloud<PointT>::ConstPtr &cloud, ros::Time stamp, std::string frame) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame;

    if (pub.getNumSubscribers() > 0) {
        pub.publish(cloud_msg);
        return true;
    } else {
        return false;
    }
}

#endif
