#include "constants.h"
#include "visualization.h"

class LidarTracking {

    ros::NodeHandle nh;

    ros::Subscriber cloud_sub;
    ros::Subscriber imu_sub;

    std::deque<sensor_msgs::Imu> imu_queue;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr current_point_cloud;
    cv::Mat range_image;
    std_msgs::Header current_cloud_header;

    pcl::PointCloud<OusterPointXYZIRT>::Ptr indexed_point_cloud;

    cv::Mat ground_image;
    cv::Mat label_image;

    // @PERFORMANCE(rahul): not sure how efficient usuing queue<pair> or vector<pair> is
    std::queue<std::pair<int, int>> segmentation_queue;
    std::vector<std::pair<int, int>> neighbors = {{-1, 0}, {0, 1}, {1, 0}, {-0, -1}}; // row, col

public:
    LidarTracking()
    : current_point_cloud(new pcl::PointCloud<OusterPointXYZIRT>()),
      indexed_point_cloud(new pcl::PointCloud<OusterPointXYZIRT>()),
      range_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_32FC1, cv::Scalar::all(0)),
      ground_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_8SC1, cv::Scalar::all(0)),
      label_image(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_32SC1, cv::Scalar::all(0)) {

        //                       topic     queue size     function              object       
        cloud_sub = nh.subscribe(lidar_topic, 1, &LidarTracking::cloud_callback, this);
        /* imu_sub   = nh.subscribe(imu_topic,   1, &LidarTracking::imu_callback,   this); */

        indexed_point_cloud->resize(VERTICAL_CHANNEL_RESOLUTION * HORIZONTAL_CHANNEL_RESOLUTION);

    }

    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    /* void imu_callback(const sensor_msgs::Imu &imu_msg); */

    void convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void project_point_cloud();
    void remove_ground();
    void preprocess_point_cloud_segmentation();
    void segment_point_cloud();
    void reset();

};

void LidarTracking::reset() {

    // reset point cloud for taking a new cloud in
    this->current_point_cloud->clear();

    // reset depth image 
    this->range_image.setTo(cv::Scalar::all(0));
    this->ground_image.setTo(cv::Scalar::all(0));

}

void LidarTracking::cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    convert_point_cloud(cloud_msg);

    project_point_cloud();

    remove_ground();

    preprocess_point_cloud_segmentation();

    segment_point_cloud();

    reset();
}

void LidarTracking::convert_point_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *current_point_cloud);
}

void LidarTracking::project_point_cloud() {

    for (const auto &point : this->current_point_cloud->points) {

        float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

        // if dist < thresh, causes numerical unstability in atan2
        if (dist < SENSOR_MIN_DIST_THRESH) {
            continue;
        }

        int row = point.ring;
        if (row < 0 || row >= VERTICAL_CHANNEL_RESOLUTION) continue;

        size_t col = round((atan2(point.y, point.x) * 180 / M_PI + 180) / HORIZONTAL_ANGULAR_RESOLUTION);
        if (col >= HORIZONTAL_CHANNEL_RESOLUTION) {
            col -= HORIZONTAL_CHANNEL_RESOLUTION;
        }

        this->range_image.at<float>(row, col) = dist;
        auto index = col + row * HORIZONTAL_CHANNEL_RESOLUTION;
        this->indexed_point_cloud->points[index] = point; // @ROBUSTNESS: does this actually copy the point?

    }

    // @DEBUG
    double max = 1;
    cv::minMaxLoc(range_image, 0, &max);
    cv::imshow("Projected Point Cloud", this->range_image / max);
    if (cv::waitKey(32) == 113) return;
}

// @Refactor different file?
cv::Mat smooth_range_image(const cv::Mat &range_image, int step, float depth_thresh) {
    cv::Mat smoothed_range_image = range_image.clone();
    for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) {
        for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) {
            float &range = smoothed_range_image.at<float>(r, c);
            if (range != FLT_MAX) {
                int counter = 0;
                float sum = 0.0f;
                
                for (int i = 1; i < step; i++) {
                    if (r - i < 0) continue;

                    for (int j = 1; j < step; j++) {
                        if (r + j > VERTICAL_CHANNEL_RESOLUTION - 1) continue;

                        const float prev_range = smoothed_range_image.at<float>(r - i, c);
                        const float next_range = smoothed_range_image.at<float>(r + j, c);
                        if (prev_range != FLT_MAX && next_range != FLT_MAX && fabs(prev_range - next_range) < depth_thresh) {
                            counter += 2;
                            sum += prev_range + next_range;
                        }
                    }
                }
                if (counter > 0) {
                    range = sum / counter;
                }
            }
        }
    }
    return smoothed_range_image;
}

void LidarTracking::remove_ground() {



/*     cv::Mat smoothed_range_image = smooth_range_image(this->range_image, 5, 1.0f); */
/*  */
/*     float max_dist = -FLT_MAX; */
/*     // @DEBUG */
/*     for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) { */
/*         for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) { */
/*             const float range = smoothed_range_image.at<float>(r, c); */
/*             if (range != FLT_MAX && range > max_dist) { */
/*                 max_dist = range; */
/*             } */
/*         } */
/*     } */

/*     cv::Mat diff = smoothed_range_image - range_image; */
/*     printf("norm: %f\n", cv::norm(diff)); */
/*  */
/*     cv::imshow("Smoothed Depth Image", smoothed_range_image / max_dist); */
/*     if (cv::waitKey(32) == 113) return; */
/*     for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) { */
/*         // do lidar rings go from top to bottom? */
/*         for (int r = VERTICAL_CHANNEL_RESOLUTION - 1; r >= VERTICAL_CHANNEL_RESOLUTION - 1 - POSSIBLE_GROUND_INDICES; r--) { */
/*  */
/*             if (range_image.at<float>(r - 1, c) == FLT_MAX || range_image.at<float>(r, c) == FLT_MAX) { */
/*                 ground_image.at<int8_t>(r, c) = -1; */
/*             } */
/*  */
/*             const auto index1 = r * HORIZONTAL_CHANNEL_RESOLUTION + c; */
/*             const auto index2 = index1 - HORIZONTAL_CHANNEL_RESOLUTION; */
/*  */
/*             const float dx = indexed_point_cloud->points[index2].x - indexed_point_cloud->points[index1].x; */
/*             const float dy = indexed_point_cloud->points[index2].y - indexed_point_cloud->points[index1].y; */
/*             const float dz = indexed_point_cloud->points[index2].z - indexed_point_cloud->points[index1].z; */
/*  */
/*             const float angle = atan2(dz, sqrt(dx * dx + dy * dy)) * 180 / M_PI; */
/*  */
/*             // @ArbitraryParameter got from LeGO LOAM */
/*             if (angle < 10) { */
/*                 ground_image.at<int8_t>(r, c) = 1;  */
/*                 ground_image.at<int8_t>(r - 1, c) = 1;  */
/*             } */
/*  */
/*         } */
    /* } */

    // @DEBUG
/*     cv::Mat ground_viz(VERTICAL_CHANNEL_RESOLUTION, HORIZONTAL_CHANNEL_RESOLUTION, CV_8UC1, cv::Scalar(255)); */
/*     for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) { */
/*         for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) { */
/*             if (ground_image.at<int8_t>(r, c) == 1) { */
/*                 ground_viz.at<uint8_t>(r, c) = 0; */
/*             } */
/*         } */
/*     } */
/*  */
/*     cv::imshow("Ground Plane", ground_viz); */
/*     if (cv::waitKey(32) == 113) return; */


}

void LidarTracking::preprocess_point_cloud_segmentation() {

    this->label_image.setTo(cv::Scalar::all(0));

    for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) {
        for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) {
            if (ground_image.at<int8_t>(r, c) == 1 || range_image.at<float>(r, c) < EPSILON) {
                label_image.at<float>(r, c) = -1;
            }
        }
    }
}

void LidarTracking::segment_point_cloud() {
    int label = 1;

    std::vector<std::pair<int, int>> current_cluster; // keep track of the points in the current cluster
    for (int r = 0; r < VERTICAL_CHANNEL_RESOLUTION; r++) {
        for (int c = 0; c < HORIZONTAL_CHANNEL_RESOLUTION; c++) {
            if (this->label_image.at<int>(r, c) != 0) continue;
            segmentation_queue.emplace(r, c);

            current_cluster.clear();
            current_cluster.emplace_back(r, c);

            while (!this->segmentation_queue.empty()) {
                const auto &index = segmentation_queue.front();
                const int r1 = index.first; const int c1 = index.second;
                this->label_image.at<int>(r1, c1) = label;

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
                        current_cluster.emplace_back(r2, c2);
                    }
                }
                segmentation_queue.pop();
            }

            // make sure we have a valid cluster
            // @Refactor 30 should be a param
            // current value got from LEGO LOAM
            if (current_cluster.size() >= 30) {
                label++;
            } else {
                for (const auto &index : current_cluster) {
                    this->label_image.at<int>(index.first, index.second) = -1;
                }
            }
        }
    }

    cv::Mat segmented_image = visualize_labels(label_image);

    cv::imshow("Segmented Image", segmented_image);
    if (cv::waitKey(32) == 113) return;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_inertial_prototype");

    LidarTracking lt;
    
    ros::spin();
    return 0;

}
