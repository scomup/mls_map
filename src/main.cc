

#include <chrono>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "viewer/viewer.h"

using namespace maps::grid;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    // ... do data processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *local_cloud);
    size_t j = 0;
    for (size_t i = 0; i < local_cloud->points.size(); ++i)
    {
        if (!pcl_isfinite(local_cloud->points[i].x) ||
            !pcl_isfinite(local_cloud->points[i].y) ||
            !pcl_isfinite(local_cloud->points[i].z))
            continue;
        local_cloud->points[j] = local_cloud->points[i];
        j++;
    }
    if (j != local_cloud->points.size())
    {
        // Resize to the correct size
        local_cloud->points.resize(j);
    }
    cloud = local_cloud;
    cloud->width = cloud->size();
}

int main(int argc, char **argv)
{
/*
    std::vector<double> data{
        1,
        1,
        1,
        1,
        2,
        2,
        2,
        2,
        2,
        2,
        2.1,
        3.1,
        3.2,
        3.3
        };
    auto kmean = auto_cluster(data,0.01);
    exit(0);
    */
    ros::init(argc, argv, "test_traversability");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    
    ros::Subscriber sub = nh.subscribe("/map3d", 1, cloud_cb);

    std::cout << "wait map3d topic\n";
    while (cloud == nullptr && ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        ros::spinOnce();
    }

    /*
    pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ p0(0., 0., 0.11);
    pcl::PointXYZ p1(0., 0., 0.12);
    pcl::PointXYZ p2(0., 0., 0.13);
    pcl::PointXYZ p3(0., 0., 1.12);
    pcl::PointXYZ p4(0., 0., 1.11);
    pcl::PointXYZ p5(0., 0., 1.12);

    local_cloud->push_back(p0);
    local_cloud->push_back(p1);
    local_cloud->push_back(p2);
    local_cloud->push_back(p3);
    local_cloud->push_back(p4);
    local_cloud->push_back(p5);
    cloud = local_cloud;
    */
    Eigen::AlignedBox2d known_cells_box;
    for (auto &point : *cloud)
    {
        known_cells_box.extend(Eigen::Vector2d(point.x, point.y));
    }

    const double grid_size = 0.1;
    Eigen::Vector2d res(grid_size, grid_size);

    double width = std::floor((known_cells_box.sizes().x()) / grid_size + 10);
    double height = std::floor((known_cells_box.sizes().y()) / grid_size + 10);

    Vector2ui numCells(width, height);
    MLSConfig mls_config;
    typedef MLSMap<MLSConfig::SLOPE> MLSMap;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMap mls(numCells, res, mls_config);

    mls.getLocalFrame().translation() << -known_cells_box.min(), 0;

    mls.CreateMapByPointCloud(*cloud);
    /*
    for (auto &point : *cloud)
    {
        mls.mergePoint(Eigen::Vector3d(point.x, point.y, point.z));
    }
    */

    std::cout << "OK!\n";

    
    //const GridMap<SPListST> &mls = *this;
    Vector2ui num_cell = mls.getNumCells();
    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            auto &list = mls.at(x, y);

            Vector2d pos(0.00, 0.00);
            // Calculate the position of the cell center.
            pos = (maps::grid::Index(x, y).cast<double>() + maps::grid::Vector2d(0.5, 0.5)).array() * mls.getResolution().array();
            //geode.setPosition(pos.x(), pos.y());
            for (auto it = list.begin(); it != list.end(); it++)
            {
                //cout <<"Mean:"<<it->getMean() <<std::endl;
                //cout <<"Variance:"<<it->getVariance() <<std::endl;
                //cout <<"Height:"<<it->getHeight() <<std::endl;
                //std::cout<<"------------------------\n";
                //std::cout <<"Min:"<<it->getMin() <<std::endl;
                //std::cout <<"Max:"<<it->getMax() <<std::endl;
                //std::cout <<"getNormal:"<<it->getNormal() <<std::endl;

            }
        }
    }

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    viewer->SetMap(&mls);
    viewer_thread.join();

    return 0;
}
