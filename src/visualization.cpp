#include "constants.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/visualization/cloud_viewer.h>


class Visualization {

    struct Viewport {
        int id = 0;
    };

    struct Cloud {
        enum Flags {
            COLOR     = 1 << 0,
        };

        enum Type {
            XYZI,
            XYZL
        };
        
        int flags = 0;
        Type type = XYZI;

        sensor_msgs::PointCloud2ConstPtr cloud_msg;
        ros::Subscriber cloud_sub;
        
        std::vector<int> viewports = {0};
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

    std::vector<Viewport> viewports;
    std::unordered_map<std::string, Cloud> clouds;

    pcl::visualization::PCLVisualizer::Ptr viewer;

public:
    Visualization(int argc, char **argv)
    : viewer(new pcl::visualization::PCLVisualizer("Viewer")) {

        /* Parse Command Line Arguments */
        int cursor = 1;
        while (cursor < argc) {
            std::string s = argv[cursor];
            if (s == "--viewports") {
                const int num_viewports = std::stoi(argv[++cursor]);
                const float width = 1.0f / num_viewports;
                for (int i = 0; i < num_viewports; i++) {
                    Viewport v;
                    viewer->createViewPort(0 + width * i, 0, 0 + width * (i + 1), 1, v.id) ;
                    viewer->setBackgroundColor(0.1f * i, 0.1f * i, 0.1f * i, v.id);
                    viewports.push_back(v);
                }
            } else if (s == "--cloud") {
                cursor += 1;
                const std::string topic = argv[cursor];
                auto &cloud = clouds[topic];
                cloud.cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(topic, 1, boost::bind(&Visualization::cloud_handler, this, _1, topic));

                cursor += 1;
                while (cursor < argc) {
                    s = argv[cursor];
                    if (s == "--type" || s == "-t") {
                        s = argv[++cursor];
                        if (s == "0" || s == "intensity") {
                            cloud.type = Cloud::XYZI;
                        } else if (s == "1" || s == "label") {
                            cloud.type = Cloud::XYZL;
                        }
                    } else if (s == "-p" || s == "--point-size") {
                        cloud.point_size = std::stoi(argv[++cursor]);
                    } else if (s == "-v" || s == "--viewport") {
                        cloud.viewports.clear();
                        std::stringstream ss(argv[++cursor]);
                        while (std::getline(ss, s, ',')) {
                            cloud.viewports.push_back(std::stoi(s));
                        }
                    } else if (s == "-c" || s == "--color") {
                        cloud.flags |= Cloud::COLOR;

                        std::stringstream ss(argv[++cursor]);
                        int i = 0;
                        while (std::getline(ss, s, ',')) {
                            cloud.color[i++] = std::stof(s);
                        }
                    } else if (s == "-o" || s == "--opacity") {
                        cloud.opacity = std::stof(argv[++cursor]);
                    } else {
                        cursor -= 1;
                        break;
                    }
                    cursor += 1;
                }
                printf("cloud=[topic=[%s], type=[%d], point_size=[%d], color=[%1.1f,%1.1f,%1.1f,], opacity=[%1.2f], viewports=[", 
                        topic.c_str(), cloud.type, cloud.point_size, cloud.r, cloud.g, cloud.b, cloud.opacity);
                for (const auto &viewport : cloud.viewports) {
                    printf("%d,", viewport);
                }
                printf("]]\n");
            }
            cursor += 1;
        }

        viewer->initCameraParameters();
        viewer->setShowFPS(false);

        if (viewports.size() == 0) {
            Viewport v{0};
            viewports.push_back(v);
        }

        for (int i = 0; i < viewports.size(); i++) {
            const auto &viewport = viewports[i];
            const auto id = std::to_string(viewport.id);
            int y = 10;
            for (const auto &cloud : clouds) {
                for (const auto &v : cloud.second.viewports) {
                    if (v == i) {
                        viewer->addText(cloud.first, 10, y, 10, cloud.second.r, cloud.second.g, cloud.second.b, id + std::to_string(y), viewport.id);
                        y += 14;
                    }
                }
            }
        }

        viewer->addCoordinateSystem(1.0);


    }

    void cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const std::string &topic_name);

    void set_cloud_properties(const Cloud &cloud, const std::string &cloud_name, int viewport);
    void add_or_update_cloud(const Cloud &cloud,  const std::string &cloud_name, int viewport);

    bool ok() {
        return !viewer->wasStopped();
    }

    void spin_once(int time) {
        viewer->spinOnce(time);
    }

    
};

void Visualization::cloud_handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const std::string &topic) {
    clouds[topic].cloud_msg = cloud_msg;
    for (const auto &viewport : clouds[topic].viewports) {
        add_or_update_cloud(clouds[topic], topic, this->viewports[viewport].id);
    }
}

void Visualization::set_cloud_properties(const Cloud &cloud, const std::string &id, int viewport) {
    this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.point_size, id, viewport);
    this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud.opacity, id, viewport);
    if (cloud.flags & Cloud::COLOR) {
        this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud.r, cloud.g, cloud.b, id, viewport);
    }
}

void Visualization::add_or_update_cloud(const Cloud &cloud, const std::string &topic, int viewport) {
    const std::string id = std::to_string(viewport) + topic;
    switch (cloud.type) {
        case Cloud::XYZI: {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*cloud.cloud_msg, *cloud_ptr);
            pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity(cloud_ptr, "intensity");
            if (!this->viewer->updatePointCloud<pcl::PointXYZI>(cloud_ptr, intensity, id)) {
                this->viewer->addPointCloud<pcl::PointXYZI>(cloud_ptr, intensity, id, viewport);
                set_cloud_properties(cloud, id, viewport);
            }
            break;
        }

        case Cloud::XYZL: {
            pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZL>);
            pcl::fromROSMsg(*cloud.cloud_msg, *cloud_ptr);
            if (!this->viewer->updatePointCloud(cloud_ptr, id)) {
                this->viewer->addPointCloud(cloud_ptr, id, viewport);
                set_cloud_properties(cloud, id, viewport);
            }
            break;
        }
    };
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualization"); 
    Visualization v(argc, argv);
    while(v.ok() && ros::ok()) {
        ros::spinOnce();
        v.spin_once(100);
    }
    return 0;
}
