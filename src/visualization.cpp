#include "constants.h"
#include <pcl/visualization/cloud_viewer.h>

class Visualization {

    ros::NodeHandle nh;

    ros::Subscriber cloud_a_sub;
    ros::Subscriber cloud_b_sub;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_a;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_b;

    std::string cloud_a_topic;
    std::string cloud_b_topic;

    int v1 = 0;
    int v2 = 0;

public:
    Visualization(const std::string &cloud_a_topic = "/full_cloud", const std::string &cloud_b_topic = "/extracted_cloud")
    : viewer(new pcl::visualization::PCLVisualizer("Viewer")),
      cloud_a(new pcl::PointCloud<pcl::PointXYZI>()),
      cloud_b(new pcl::PointCloud<pcl::PointXYZI>()),
      cloud_a_topic(cloud_a_topic), cloud_b_topic(cloud_b_topic) {

        cloud_a_sub = nh.subscribe(cloud_a_topic, 1, &Visualization::cloud_a_handler, this);
        cloud_b_sub = nh.subscribe(cloud_b_topic, 1, &Visualization::cloud_b_handler, this);

        viewer->initCameraParameters();

        viewer->createViewPort(0, 0, 0.5, 1, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText(cloud_a_topic, 10, 10, "v1 title", v1);

        viewer->createViewPort(0.5, 0, 1, 1, v2);
        viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
        viewer->addText(cloud_b_topic, 10, 10, "v2 title", v2);

        viewer->addCoordinateSystem(1.0);
    }

    void cloud_a_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void cloud_b_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void add_or_update_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const std::string &cloud_name, int viewport);

    bool ok() {
        return !viewer->wasStopped();
    }

    void spin_once(int time) {
        viewer->spinOnce(time);
    }

    
};

void Visualization::cloud_a_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *cloud_a);
    add_or_update_cloud(cloud_a, cloud_a_topic, v1);

}


void Visualization::cloud_b_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *cloud_b);
    add_or_update_cloud(cloud_b, cloud_b_topic, v2);
}

void Visualization::add_or_update_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const std::string &cloud_name, int viewport) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud, "intensity");
    if (!this->viewer->updatePointCloud<pcl::PointXYZI>(cloud, handler, cloud_name)) {
        this->viewer->addPointCloud<pcl::PointXYZI>(cloud, handler, cloud_name, viewport);
    }
}

int loop(Visualization &v) {
    while(v.ok() && ros::ok()) {
        ros::spinOnce();
        v.spin_once(100);
    }
    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualization"); 
    if (argc > 2) {
        Visualization v(argv[1], argv[2]);
        return loop(v);
    } else {
        Visualization v;
        return loop(v);
    }
}
