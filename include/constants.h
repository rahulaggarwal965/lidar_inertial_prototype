#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <pcl/register_point_struct.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <string>

// Newer College Dataset

const std::string imu_topic = "/os1_cloud_node/imu";
const std::string lidar_topic = "/os1_cloud_node/points";

// OUSTER OS1-64
const int VERTICAL_CHANNEL_RESOLUTION = 64;
const int HORIZONTAL_CHANNEL_RESOLUTION = 1024;
const float VERTICAL_ANGULAR_RESOLUTION = 33.2 / float(VERTICAL_CHANNEL_RESOLUTION);
const float HORIZONTAL_ANGULAR_RESOLUTION = 360.0 / float(HORIZONTAL_CHANNEL_RESOLUTION);
const float VERTICAL_ALPHA = VERTICAL_ANGULAR_RESOLUTION / 180 * M_PI;
const float HORIZONTAL_ALPHA = HORIZONTAL_ANGULAR_RESOLUTION / 180 * M_PI;

// ground removal indices
const int POSSIBLE_GROUND_INDICES = 9;

// segmentation threshold
const float SEG_THRESH = 10.0 / 180 * M_PI; // 10 degrees according to paper

// sensor distance threshold
const float SENSOR_MIN_DIST_THRESH = 1.0f;
const float SENSOR_MAX_DIST_THRESH = 1000.0f;

const float EPSILON = 0.001f;


// OUSTER point type
struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (OusterPointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range)
)

#endif
