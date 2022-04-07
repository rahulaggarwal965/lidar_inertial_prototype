#include "constants.h"
#include <pcl/visualization/cloud_viewer.h>

class Visualization {

    ros::NodeHandle nh;

    ros::Subscriber extracted_cloud_sub;
    ros::Subscriber full_cloud_sub;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    pcl::PointCloud<pcl::PointXYZI>::Ptr extracted_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr full_cloud;

    int v1 = 0;
    int v2 = 0;

public:
    Visualization()
    : viewer(new pcl::visualization::PCLVisualizer("Viewer")),
      extracted_cloud(new pcl::PointCloud<pcl::PointXYZI>()),
      full_cloud(new pcl::PointCloud<pcl::PointXYZI>()) {

        extracted_cloud_sub = nh.subscribe("/extracted_cloud", 1, &Visualization::extracted_cloud_handler, this);
        full_cloud_sub      = nh.subscribe("/full_cloud",      1, &Visualization::full_cloud_handler,      this);

        viewer->initCameraParameters();

        viewer->createViewPort(0, 0, 0.5, 1, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->addText("Extracted Cloud", 10, 10, "v1 title", v1);

        viewer->createViewPort(0.5, 0, 1, 1, v2);
        viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
        viewer->addText("Full Cloud", 10, 10, "v2 title", v2);

        viewer->addCoordinateSystem(1.0);
    }

    void extracted_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void full_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void add_or_update_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const std::string &cloud_name, int viewport);

    bool ok() {
        return !viewer->wasStopped();
    }

    void spin_once(int time) {
        viewer->spinOnce(time);
    }

    
};

void Visualization::extracted_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *extracted_cloud);
    add_or_update_cloud(extracted_cloud, "extracted_cloud", v1);

}


void Visualization::full_cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    pcl::fromROSMsg(*cloud_msg, *full_cloud);
    add_or_update_cloud(full_cloud, "full_cloud", v2);
}

void Visualization::add_or_update_cloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, const std::string &cloud_name, int viewport) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler(cloud, "intensity");
    if (!this->viewer->updatePointCloud<pcl::PointXYZI>(cloud, handler, cloud_name)) {
        this->viewer->addPointCloud<pcl::PointXYZI>(cloud, handler, cloud_name, viewport);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualization"); 
    Visualization v;
    while (v.ok() && ros::ok()) {
        ros::spinOnce();
        v.spin_once(100);
    }
    return 0;
}
