#include "constants.h"

class LidarTracking {

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Subscriber imu_sub;

    std::deque<sensor_msgs::Imu> imu_queue;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr current_point_cloud;
    cv::Mat range_image;
    std_msgs::Header current_cloud_header;

public:
    LidarTracking()
    : current_point_cloud(new pcl::PointCloud<OusterPointXYZIRT>()),
      range_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_32FC1, cv::Scalar::all(FLT_MAX)) {

        //                       topic     queue size     function              object       
        cloud_sub = nh.subscribe(lidar_topic, 1, &LidarTracking::cloud_callback, this);
        /* imu_sub   = nh.subscribe(imu_topic,   1, &LidarTracking::imu_callback,   this); */

    }

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    /* void imu_callback(const sensor_msgs::Imu &imu_msg); */

    void convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void project_point_cloud();

};

void LidarTracking::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    convert_point_cloud(cloud_msg);
}

void LidarTracking::convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *current_point_cloud);
}

void LidarTracking::project_point_cloud() {

    for (auto &point : this->current_point_cloud->points) {
        int row = point.ring;
        if (row < 0 || row >= VERTICAL_CHANNEL_RESOLUTION) continue;

        // @ROBUSTNESS(rahul): add check for column out of bound
        int col = round((atan2(point.y, point.x) * 180 / M_PI + 180) / HORIZONTAL_ANGULAR_RESOLUTION);

        float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        // add check for minimum range
        range_image.at<float>(row, col) = dist;



    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_inertial_prototype");

    LidarTracking lt;
    
    ros::spin();
    return 0;

}
