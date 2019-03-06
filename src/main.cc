

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include "src/graph/dijkstra_graph.h"
#include "src/graph/astar_graph.h"

#include "viewer/viewer.h"

using namespace maps::grid;
using namespace GraphNavigation::Graph;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
MLSMap<MLSConfig::SLOPE>* mls_;
DijkstraGraph<Eigen::Vector3d, float>* graph_;
Eigen::Vector3d start_(-1,-1,-1);
Eigen::Vector3d goal_(-1,-1,-1);
ros::Publisher g_plan_pub_;
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    Eigen::Vector3d pos(x,y,z);
    Index idx;
    Eigen::Vector3d pos_in_cell;

    if (!mls_->toGrid(pos, idx, pos_in_cell))
        return;
    auto &list = mls_->at(idx.x(), idx.y());
    double min_diff_top = 0;
    double min_diff  = 100;
    if(list.size() == 0)
        return;
    for (auto it = list.begin(); it != list.end(); it++)
    {
        double top = it->getTop();
        double diff = std::abs(top - z);
        if( min_diff >  diff){
            min_diff_top = top;
            min_diff = diff;
        }
    }
    mls_->fromGrid(maps::grid::Index(x, y), pos);
    goal_ = Eigen::Vector3d(pos.x(), pos.y(), min_diff_top);
    std::cout<<"get goal point!"<<std::endl;
    std::cout<<goal_<<std::endl;

    if (goal_.x() == -1)
    {
        std::cout << "we need a goal!" << std::endl;
        return;
    }
    if (start_.x() == -1)
    {
        std::cout << "we need a start!" << std::endl;
        return;
    }
    std::vector<Eigen::Vector3d> path;
    nav_msgs::Path path_msg;

    graph_->FindPath(start_, goal_, path);

    path_msg.poses.resize(path.size());
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();


    //graph_->FindPath(start_idx, goal_idx, std::vector<uint> path);
    int i = 0;
    for (Eigen::Vector3d p : path)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.position.z = p.z();
        path_msg.poses[i++] = pose;
    }
    g_plan_pub_.publish(path_msg);
    
    


  }

void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    Eigen::Vector3d pos(x,y,z);
    Index idx;
    Eigen::Vector3d pos_in_cell;

    if (!mls_->toGrid(pos, idx, pos_in_cell))
        return;
    auto &list = mls_->at(idx.x(), idx.y());
    double min_diff_top = 0;
    double min_diff  = 100;
    if(list.size() == 0)
        return;
    for (auto it = list.begin(); it != list.end(); it++)
    {
        
        double top = it->getTop();
        printf("top %f\n",top);
        double diff = std::abs(top - z);
        if( min_diff >  diff){
            min_diff_top = top;
            min_diff = diff;
        }
    }
    mls_->fromGrid(maps::grid::Index(x, y), pos);
    start_ = Eigen::Vector3d(pos.x(), pos.y(), min_diff_top);
    std::cout<<"get start point!"<<std::endl;
    std::cout<<start_<<std::endl;
}

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

    ros::Subscriber sub = nh.subscribe("map3d", 1, cloud_cb);
    ros::Subscriber initial_pose_sub = nh.subscribe("initialpose", 2, initialPoseReceived);
    ros::Subscriber goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, goalCB);
    g_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
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
    //Make map
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
    mls_ = &mls;

    std::cout << "Create mls map OK !\n";

    DijkstraGraph<Eigen::Vector3d, float> graph(
        [](Eigen::Vector3d state) {
             size_t seed = 0;
            boost::hash_combine(seed, state.x());
            boost::hash_combine(seed, state.y());
            boost::hash_combine(seed, state.z());
            return seed; });

    Vector2ui num_cell = mls.getNumCells();
    for (size_t x = 0; x < num_cell.x(); x++)
    {
        for (size_t y = 0; y < num_cell.y(); y++)
        {
            auto &list = mls.at(x, y);

            for (auto it = list.begin(); it != list.end(); it++)
            {
                double top = it->getTop();
                Vector3d pos(0, 0, 0);
                mls.fromGrid(maps::grid::Index(x, y), pos);
                graph.AddNode(Eigen::Vector3d(pos.x(), pos.y(), top));
            }
        }
    }
    std::cout << "Add nodes to navi graph OK!\n";


    for (auto &m : graph.graph())
    {
        const auto &pos = graph.positions()[m.first];
        Index idx;
        Eigen::Vector3d pos_in_cell;

        if (!mls.toGrid(pos, idx, pos_in_cell))
            continue;

        for (int x = idx.x() - 1; x <= idx.x() + 1; x++)
        {
            for (int y = idx.y() - 1; y <= idx.y() + 1; y++)
            {
                if (x == idx.x() && y == idx.y())
                    continue;
                if (!mls.inGrid(Index(x, y)))
                    continue;
                auto &list = mls.at(x, y);
                for (auto it = list.begin(); it != list.end(); it++)
                {
                    Vector3d near_pos(0, 0, 0);
                    if (!mls.fromGrid(maps::grid::Index(x, y), near_pos))
                        continue;
                    double top = it->getTop();

                    if (std::abs(top - pos.z()) < 0.1)
                    {
                        graph.AddEdge(pos, Eigen::Vector3d(near_pos.x(), near_pos.y(), top));
                    }

                }
            }
        }
    }
    graph_ = &graph;
    std::cout << "Add edges to navi graph OK!\n";

    auto viewer = new Viewer();
    auto viewer_thread = std::thread(&Viewer::Run, viewer);

    viewer->SetMap(&mls);
    viewer->SetGraph(&graph);
    ros::spin();
    //viewer_thread.join();

    return 0;
}
