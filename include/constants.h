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

// ground removal indices
const int POSSIBLE_GROUND_INDICES = 9;

// segmentation threshold
const float GROUND_ANGLE_THRESH = 5.0 / 180 * M_PI;
const float VERTICAL_ALPHA = VERTICAL_ANGULAR_RESOLUTION / 180 * M_PI;
const float HORIZONTAL_ALPHA = HORIZONTAL_ANGULAR_RESOLUTION / 180 * M_PI;
const float SEG_THRESH = 10.0 / 180 * M_PI; // 10 degrees according to paper

// sensor distance threshold
const float SENSOR_MIN_DIST_THRESH = 1.0f;
const float SENSOR_MAX_DIST_THRESH = 1000.0f;

// removal of degenerate points
const float OCCLUSION_THRESH = 0.3f;
const float PARALLEL_BEAM_MULTIPLIER = 0.2f;

const float CURVATURE_EDGE_THRESH  = 1.0; // @ArbitraryParameter
const float CURVATURE_PLANE_THRESH = 0.1; // @ArbitraryParameter

// number of regions the horizontal fov gets cut up into
// when doing feature extraction. Must divide into
// HORIZONTAL_CHANNEL_RESOLUTION
const int NUM_FEATURE_EXTRACTION_REGIONS = 4;
const int NUM_EDGE_FEATURES_PER_REGION = 2; // from LOAM
const int NUM_PLANAR_FEATURES_PER_REGION = 4; // from LOAM

const float EPSILON = 0.001f;

const bool VISUALIZE_SEG = false;


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
