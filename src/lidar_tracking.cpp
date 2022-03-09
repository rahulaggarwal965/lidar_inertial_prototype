#include "constants.h"

class LidarTracking {

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Subscriber imu_sub;

    std::deque<sensor_msgs::Imu> imu_queue;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr current_point_cloud;
    cv::Mat range_image;
    std_msgs::Header current_cloud_header;

    cv::Mat label_image;
    // @PERFORMANCE(rahul): not sure how efficient usuing queue<pair> or vector<pair> is
    std::queue<std::pair<int, int>> segmentation_queue;
    std::vector<std::pair<int, int>> neighbors = {{-1, 0}, {0, 1}, {1, 0}, {-0, -1}}; // row, col

public:
    LidarTracking()
    : current_point_cloud(new pcl::PointCloud<OusterPointXYZIRT>()),
      range_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_32FC1, cv::Scalar::all(FLT_MAX)),
      label_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_32SC1, cv::Scalar::all(0)) {

        //                       topic     queue size     function              object       
        cloud_sub = nh.subscribe(lidar_topic, 1, &LidarTracking::cloud_callback, this);
        /* imu_sub   = nh.subscribe(imu_topic,   1, &LidarTracking::imu_callback,   this); */

    }

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    /* void imu_callback(const sensor_msgs::Imu &imu_msg); */

    void convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void project_point_cloud();
    void preprocess_point_cloud_segmentation();
    void segment_point_cloud();

};

void LidarTracking::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    convert_point_cloud(cloud_msg);
    project_point_cloud();
    preprocess_point_cloud_segmentation();
    segment_point_cloud();
}

void LidarTracking::convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *current_point_cloud);
}

void LidarTracking::project_point_cloud() {

    // @DEBUG
    float max_dist = -FLT_MAX;

    for (auto &point : this->current_point_cloud->points) {

        float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // if dist < thresh, causes numerical unstability in atan2
        if (dist < SENSOR_DIST_THRESH) continue;

        int row = point.ring;
        if (row < 0 || row >= VERTICAL_CHANNEL_RESOLUTION) continue;

        // @ROBUSTNESS(rahul): add check for column out of bound
        size_t col = round((atan2(point.y, point.x) * 180 / M_PI + 180) / HORIZONTAL_ANGULAR_RESOLUTION);
        if (col >= HORIZONTAL_CHANNEL_RESOLUTION) {
            col -= HORIZONTAL_CHANNEL_RESOLUTION;
        }

        this->range_image.at<float>(row, col) = dist;

        if (dist > max_dist) {
            max_dist = dist;
        }

    }

    // @DEBUG
    printf("max dist: %f\n", max_dist);
    cv::imshow("Projected Point Cloud", this->range_image / max_dist);
    if (cv::waitKey(32) == 113) return;
}

void LidarTracking::preprocess_point_cloud_segmentation() {

    this->label_image.setTo(cv::Scalar::all(0));

    for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) {
        for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) {
            if (range_image.at<float>(r, c) == FLT_MAX) {
                label_image.at<float>(r, c) = -1;
            }
        }
    }
}

void LidarTracking::segment_point_cloud() {
    int label = 0;
    for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) {
        for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) {
            printf("r: %d, c: %d\n", r, c);
            if (this->label_image.at<int>(r, c) != 0) continue;
            this->label_image.at<int>(r, c) = label;
            segmentation_queue.emplace(r, c);
            while (!this->segmentation_queue.empty()) {
                const auto &index = segmentation_queue.front();
                const int r1 = index.first; const int c1 = index.second;

                for (const auto &neighbor_offset : neighbors) {
                    int r2 = r1 + neighbor_offset.first;
                    int c2 = c1 + neighbor_offset.second;

                    if (r2 < 0 || r2 >= VERTICAL_CHANNEL_RESOLUTION) continue; // vfov is not 360 degress
                    if (c2 < 0) {
                        c2 = HORIZONTAL_CHANNEL_RESOLUTION - 1;
                    } else if (c2 >= HORIZONTAL_CHANNEL_RESOLUTION) {
                        c2 = 0;
                    }

                    if (label_image.at<int>(r2, c2) != 0) continue;

                    float d1 = fmax(range_image.at<float>(r1, c1), range_image.at<float>(r2, c2));
                    float d2 = fmin(range_image.at<float>(r1, c1), range_image.at<float>(r2, c2));

                    float alpha = (neighbor_offset.first == 0) ? HORIZONTAL_ALPHA : VERTICAL_ALPHA;

                    if (atan2(d2 * sin(alpha), d1 - d2 * cos(alpha)) > SEG_THRESH) {
                        this->label_image.at<int>(r2, c2) = label;
                        segmentation_queue.emplace(r2, c2);
                    }
                }
                segmentation_queue.pop();
            }
            label++;
        }
    }

    cv::imshow("Segmented Image", this->label_image);
    if (cv::waitKey(32) == 113) return;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_inertial_prototype");

    LidarTracking lt;
    
    ros::spin();
    return 0;

}
