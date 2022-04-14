#include "constants.h"
#include <pcl/visualization/cloud_viewer.h>


class Visualization {

    struct Cloud {
        enum Flags {
            COLOR   = 1 << 0,
        };
        
        int flags = 0;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
        ros::Subscriber cloud_sub;
        
        int viewport = 0;
        int point_size = 1;
        float opacity = 1.0f;
        union {
            struct {
                float r = 1.0f;
                float g = 1.0f;
                float b = 1.0f;
            };
            float color[3];
        };
    };

    ros::NodeHandle nh;

    int num_viewports = 1;
    std::unordered_map<std::string, Cloud> clouds;

    pcl::visualization::PCLVisualizer::Ptr viewer;

    int v1 = 0;
    int v2 = 0;

public:
    Visualization(int argc, char **argv)
    : viewer(new pcl::visualization::PCLVisualizer("Viewer")) {

        int cursor = 1;
        while (cursor < argc) {
            std::string s = argv[cursor];
            if (s == "--cloud") {
                std::string topic = argv[++cursor];
                auto &cloud = clouds[topic];
                cloud.cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(&Visualization::cloud_handler, this, _1, topic));
                cloud.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

                while (++cursor < argc) {
                    s = argv[cursor];
                    if (s == "--point-size" || s == "-p") {
                        cloud.point_size = std::stoi(argv[++cursor]);
                    } else if (s == "--viewport" || s == "-v") {
                        cloud.viewport = std::stoi(argv[++cursor]);
                        if (cloud.viewport > 0) {
                            num_viewports = 2;
                        }
                    } else if (s == "--color" || s == "-c") {
                        cloud.flags |= Cloud::COLOR;
                        std::stringstream ss(argv[++cursor]);
                        int i = 0;
                        while (std::getline(ss, s, ',')) {
                            cloud.color[i++] = std::stof(s);
                        }
                    } else if (s == "--opacity" || s == "-o") {
                        cloud.opacity = std::stof(argv[++cursor]);
                    } else {
                        break;
                    }
                }
            }
        }

        viewer->initCameraParameters();
        viewer->setShowFPS(false);

        if (num_viewports == 1) {
            viewer->setBackgroundColor(0, 0, 0);
            int y = 10;
            for (const auto &cloud : clouds) {
                viewer->addText(cloud.first, 10, y, 10, cloud.second.r, cloud.second.g, cloud.second.b, std::to_string(y));
                y += 14;
            }
        } else if (num_viewports == 2) {
            viewer->createViewPort(0, 0, 0.5, 1, v1);
            viewer->setBackgroundColor(0, 0, 0, v1);

            int y = 10;
            for (const auto &cloud : clouds) {
                if (cloud.second.viewport == 0) {
                    viewer->addText(cloud.first, 10, y, 10, cloud.second.r, cloud.second.g, cloud.second.b, "1" + std::to_string(y));
                    y += 14;
                }
            }

            viewer->createViewPort(0.5, 0, 1, 1, v2);
            viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);

            y = 10;
            for (const auto &cloud : clouds) {
                if (cloud.second.viewport == 1) {
                    viewer->addText(cloud.first, 10, y, 10, cloud.second.r, cloud.second.g, cloud.second.b, "2" + std::to_string(y));
                    y += 14;
                }
            }
        }

        viewer->addCoordinateSystem(1.0);


    }

    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const std::string &topic_name);

    void add_or_update_cloud(const Cloud &cloud, const std::string &cloud_name);

    bool ok() {
        return !viewer->wasStopped();
    }

    void spin_once(int time) {
        viewer->spinOnce(time);
    }

    
};

void Visualization::cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const std::string &topic) {
    pcl::fromROSMsg(*cloud_msg, *clouds[topic].cloud);
    add_or_update_cloud(clouds[topic], topic);
}

void Visualization::add_or_update_cloud(const Cloud &cloud, const std::string &topic) {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(cloud.cloud, "intensity");
    if (!this->viewer->updatePointCloud<pcl::PointXYZI>(cloud.cloud, intensity, topic)) {
        int viewport = (cloud.viewport) ? v2 : v1;
        this->viewer->addPointCloud<pcl::PointXYZI>(cloud.cloud, intensity, topic, viewport);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.point_size, topic, viewport);
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.opacity, topic, viewport);
        if (cloud.flags & Cloud::COLOR) {
            this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud.r, cloud.g, cloud.b, topic, viewport);
        }
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
    Visualization v(argc, argv);
    return loop(v);
}
