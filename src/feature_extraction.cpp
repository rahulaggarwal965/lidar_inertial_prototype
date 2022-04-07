#include "constants.h"
#include "helpers.h"
#include "lidar_inertial_prototype/segmentation_info.h"

// need this for std::sort
struct curvature_index_pair {
    float curvature;
    int index;
};

class FeatureExtraction {

    ros::NodeHandle nh;

    ros::Subscriber segmentation_info_sub;
    ros::Subscriber extracted_cloud_sub;

    lidar_inertial_prototype::segmentation_info segmentation_info;
    pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud;

    std::vector<curvature_index_pair> curvature_index_pairs;
    std::vector<int> invalid_points;

    pcl::PointCloud<pcl::PointXYZI>::Ptr curvature_cloud;
    ros::Publisher curvature_cloud_pub;

    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_feature_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_feature_cloud;

    ros::Publisher edge_feature_cloud_pub;
    ros::Publisher plane_feature_cloud_pub;

public:
    FeatureExtraction()
    : extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>()),
      curvature_index_pairs(VERTICAL_CHANNEL_RESOLUTION * HORIZONTAL_CHANNEL_RESOLUTION),
      invalid_points(VERTICAL_CHANNEL_RESOLUTION * HORIZONTAL_CHANNEL_RESOLUTION),
      curvature_cloud(new pcl::PointCloud<pcl::PointXYZI>()),
      edge_feature_cloud(new pcl::PointCloud<pcl::PointXYZI>()), 
      plane_feature_cloud(new pcl::PointCloud<pcl::PointXYZI>()) {

        //                                    topic           queue size     function                                object       
        segmentation_info_sub = nh.subscribe("/segmentation_info", 1, &FeatureExtraction::segmentation_info_callback, this);

        //                                            type                   topic     queue size
        curvature_cloud_pub     = nh.advertise<sensor_msgs::PointCloud2>("/curvature_cloud",     1);
        edge_feature_cloud_pub  = nh.advertise<sensor_msgs::PointCloud2>("/edge_feature_cloud",  1);
        plane_feature_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/plane_feature_cloud", 1);
    }

    void segmentation_info_callback(const lidar_inertial_prototype::segmentation_infoConstPtr &segmentation_info);

    void compute_curvature();
    void remove_degenerate_features();
    void extract_features();
    void publish_feature_clouds();

};


void FeatureExtraction::segmentation_info_callback(const lidar_inertial_prototype::segmentation_infoConstPtr &segmentation_info) {
    this->segmentation_info = *segmentation_info;
    pcl::fromROSMsg(segmentation_info->extracted_cloud, *extracted_cloud);

    compute_curvature();

    remove_degenerate_features();
    /*  */
    /* extract_features(); */

    /* publish_feature_clouds(); */

    // TODO rough outline of plan
    // propagate_features_forward();
    
    // segment ... 


}

// @Refactor I'm pretty sure this calculates curvature across scan boundaries
// It should be fine when we calculate occluded points, but I should clean this up later
void FeatureExtraction::compute_curvature() {
    int num_points = extracted_cloud->points.size();

    this->curvature_cloud->clear();

    for (int i = 5; i < num_points - 5; i++) {
        const float diff = segmentation_info.point_depth[i - 5] + segmentation_info.point_depth[i - 4]
                         + segmentation_info.point_depth[i - 3] + segmentation_info.point_depth[i - 2]
                         + segmentation_info.point_depth[i - 1] - segmentation_info.point_depth[i] * 10
                         + segmentation_info.point_depth[i + 1] + segmentation_info.point_depth[i + 2]
                         + segmentation_info.point_depth[i + 3] + segmentation_info.point_depth[i + 4]
                         + segmentation_info.point_depth[i + 5];            

        /* const float diff_x = extracted_cloud->points[i - 5].x + extracted_cloud->points[i - 4].x */
        /*                    + extracted_cloud->points[i - 3].x + extracted_cloud->points[i - 2].x */
        /*                    + extracted_cloud->points[i - 1].x - 10 * extracted_cloud->points[i].x */
        /*                    + extracted_cloud->points[i + 1].x + extracted_cloud->points[i + 2].x */
        /*                    + extracted_cloud->points[i + 3].x + extracted_cloud->points[i + 4].x */
        /*                    + extracted_cloud->points[i + 5].x; */
        /* const float diff_y = extracted_cloud->points[i - 5].y + extracted_cloud->points[i - 4].y */
        /*                    + extracted_cloud->points[i - 3].y + extracted_cloud->points[i - 2].y */
        /*                    + extracted_cloud->points[i - 1].y - 10 * extracted_cloud->points[i].y */
        /*                    + extracted_cloud->points[i + 1].y + extracted_cloud->points[i + 2].y */
        /*                    + extracted_cloud->points[i + 3].y + extracted_cloud->points[i + 4].y */
        /*                    + extracted_cloud->points[i + 5].y; */
        /* const float diff_z = extracted_cloud->points[i - 5].z + extracted_cloud->points[i - 4].z */
        /*                    + extracted_cloud->points[i - 3].z + extracted_cloud->points[i - 2].z */
        /*                    + extracted_cloud->points[i - 1].z - 10 * extracted_cloud->points[i].z */
        /*                    + extracted_cloud->points[i + 1].z + extracted_cloud->points[i + 2].z */
        /*                    + extracted_cloud->points[i + 3].z + extracted_cloud->points[i + 4].z */
        /*                    + extracted_cloud->points[i + 5].z; */

        const float curvature = diff * diff;
        /* const float curvature = diff_x * diff_x + diff_y * diff_y + diff_z * diff_z; */

        this->curvature_index_pairs[i].curvature = curvature;
        this->curvature_index_pairs[i].index = i;

        /* auto &point = this->curvature_cloud->emplace_back(this->extracted_cloud->points[i]); */
        /* point.intensity = curvature; */

        this->invalid_points[i] = 0; // point is not invalid yet

    }

    /* std::sort(curvature_index_pairs.begin(), curvature_index_pairs.end(), */
    /*         [](const curvature_index_pair &k1, const curvature_index_pair &k2) -> bool {return k1.curvature < k2.curvature;}); */

    /* for (int i = 0; i < curvature_index_pairs.size(); i++) { */
    /*     const int index = curvature_index_pairs[i].index; */
    /*     this->curvature_cloud->points[index].intensity = i; */
    /* } */

    /* publish_cloud<pcl::PointXYZI>(curvature_cloud_pub, curvature_cloud, segmentation_info.header.stamp, "os1_lidar"); */
}

void FeatureExtraction::remove_degenerate_features() {
    int num_points = extracted_cloud->points.size();

    for (int i = 5; i < num_points - 6; i++) {
        const float depth1 = segmentation_info.point_depth[i];
        const float depth2 = segmentation_info.point_depth[i+1];

        const int col_diff = abs(int(segmentation_info.point_column[i+1] - segmentation_info.point_column[i]));

        if (col_diff < 10) {
            if (depth1 - depth2 > OCCLUSION_THRESH) {
                this->invalid_points[i - 5] = 1;
                this->invalid_points[i - 4] = 1;
                this->invalid_points[i - 3] = 1;
                this->invalid_points[i - 2] = 1;
                this->invalid_points[i - 1] = 1;
                this->invalid_points[i]     = 1;
            } else if (depth2 - depth1 > OCCLUSION_THRESH) {
                this->invalid_points[i + 1] = 1;
                this->invalid_points[i + 2] = 1;
                this->invalid_points[i + 3] = 1;
                this->invalid_points[i + 4] = 1;
                this->invalid_points[i + 5] = 1;
            }
        }

        const float depth_diff1 = abs(segmentation_info.point_depth[i-1] - depth1);
        const float depth_diff2 = abs(depth1 - depth2);

        if (depth_diff1 > PARALLEL_BEAM_MULTIPLIER * depth1 && depth_diff2 > PARALLEL_BEAM_MULTIPLIER * depth2) {
            this->invalid_points[i] = 1;
        }
    }

    std::sort(curvature_index_pairs.begin(), curvature_index_pairs.end(),
            [](const curvature_index_pair &k1, const curvature_index_pair &k2) -> bool {return k1.curvature < k2.curvature;});

    /* for (int i = 0; i < curvature_index_pairs.size(); i++) { */
    /*     const int index = curvature_index_pairs[i].index; */
    /*     this->curvature_cloud->points[index].intensity = i; */
    /* } */

    int k = 0;
    for (int i = 0; i < curvature_index_pairs.size(); i++) {
        const int index = curvature_index_pairs[i].index;
        if (invalid_points[index] == 0) {
            auto &point = curvature_cloud->emplace_back(extracted_cloud->points[index]);
            point.intensity = k;
            k++;
        }
    }

    
    publish_cloud<pcl::PointXYZI>(curvature_cloud_pub, curvature_cloud, segmentation_info.header.stamp, "os1_lidar");
}

void FeatureExtraction::extract_features() {

    edge_feature_cloud->clear();
    plane_feature_cloud->clear();

    for (int j = 0; j < VERTICAL_CHANNEL_RESOLUTION; j++) {
        for (int i = 0; i < NUM_FEATURE_EXTRACTION_REGIONS; i++) {

            int start_idx = segmentation_info.ring_start_idx[j] 
                + (segmentation_info.ring_end_idx[j] - segmentation_info.ring_start_idx[j]) * i / NUM_FEATURE_EXTRACTION_REGIONS;
            int end_idx   = segmentation_info.ring_start_idx[j]
                + (segmentation_info.ring_end_idx[j] - segmentation_info.ring_start_idx[j] * (i + 1) / NUM_FEATURE_EXTRACTION_REGIONS) - 1;

            if (start_idx >= end_idx) continue;

            std::sort(curvature_index_pairs.begin() + start_idx, curvature_index_pairs.begin() + end_idx, 
                    [](const curvature_index_pair &k1, const curvature_index_pair &k2) -> bool {return k1.curvature < k2.curvature;});

            int num_edge_features = 0;
            for (int k = end_idx; k >= start_idx; k--) {
                const auto &ci = curvature_index_pairs[k];
                if (invalid_points[ci.index] == 0 && ci.curvature > CURVATURE_EDGE_THRESH) {

                    if (num_edge_features < NUM_EDGE_FEATURES_PER_REGION) {
                        edge_feature_cloud->push_back(extracted_cloud->points[ci.index]);
                    } else {
                        break;
                    }
                    num_edge_features++;

                    //invalidate other points in the set
                    for (int i = -5; i < 0; i++) {
                        if (abs(int(segmentation_info.point_column[ci.index + i] - segmentation_info.point_column[ci.index])) > 10) break;
                        invalid_points[ci.index + i] = 1;
                    }
                    for (int i = 1; i < 6; i++) {
                        if (abs(int(segmentation_info.point_column[ci.index + i] - segmentation_info.point_column[ci.index])) > 10) break;
                        invalid_points[ci.index + i] = 1;
                    }
                }
            }

            int num_plane_features = 0;
            for (int k = start_idx; j <= end_idx; k++) {
                const auto &ci = curvature_index_pairs[k];
                if (invalid_points[ci.index] == 0 && ci.curvature < CURVATURE_PLANE_THRESH) {

                    if (num_plane_features < NUM_PLANAR_FEATURES_PER_REGION) {
                        plane_feature_cloud->push_back(extracted_cloud->points[ci.index]);
                    } else {
                        break;
                    }
                    num_plane_features++;

                    //invalidate other points in the set
                    for (int i = -5; i < 0; i++) {
                        if (abs(int(segmentation_info.point_column[ci.index + i] - segmentation_info.point_column[ci.index + i + 1])) > 10) break;
                        invalid_points[ci.index + i] = 1;
                    }
                    for (int i = 1; i < 6; i++) {
                        if (abs(int(segmentation_info.point_column[ci.index + i] - segmentation_info.point_column[ci.index + i + 1])) > 10) break;
                        invalid_points[ci.index + i] = 1;
                    }
                }
            }

        }
    }
}

void FeatureExtraction::publish_feature_clouds() {
    sensor_msgs::PointCloud2 temp_cloud;
    pcl::toROSMsg(*edge_feature_cloud, temp_cloud);
    temp_cloud.header.stamp = segmentation_info.header.stamp;
    temp_cloud.header.frame_id = segmentation_info.header.frame_id;
    edge_feature_cloud_pub.publish(temp_cloud);

    pcl::toROSMsg(*plane_feature_cloud, temp_cloud);
    plane_feature_cloud_pub.publish(temp_cloud);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_extraction");

    FeatureExtraction fe;

    ros::spin();
    return 0;
}
