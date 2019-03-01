

#include "MLSMap.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace maps::grid;
int main(int argc, char** argv)
{
    Eigen::Vector2d res(0.05, 0.05);
    Vector2ui numCells(300, 300);

    MLSConfig mls_config;
    typedef MLSMap<MLSConfig::SLOPE> MLSMap;
    mls_config.updateModel = MLSConfig::SLOPE;
    MLSMap mls(numCells, res, mls_config);

    mls.getLocalFrame().translation() << 0.5 * mls.getSize(), 0;

    Eigen::Vector2d max = 0.5 * mls.getSize();
    Eigen::Vector2d min = -0.5 * mls.getSize();
    for (double x = min.x(); x < max.x(); x += 0.00625)
    {
        double cs = std::cos(x * M_PI / 2.5);
        for (double y = min.y(); y < max.y(); y += 0.00625)
        {
            double sn = std::sin(y * M_PI / 2.5);
            mls.mergePoint(Eigen::Vector3d(x, y, cs * sn));
        }
    }

/*
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
                //PatchVisualizer::visualize(geode, *it);
                printf("%d %d \n", x, y);
            }
        }
    }*/

  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
// %Tag(MARKER_INIT)%
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
// %EndTag(MARKER_INIT)%

// %Tag(ID)%
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
// %EndTag(ID)%

// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
// %EndTag(TYPE)%

// %Tag(SCALE)%
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;
// %EndTag(SCALE)%

// %Tag(COLOR)%
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(HELIX)%
    // Create the vertices for the points and lines




    
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
                      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = it->getTop();
      points.points.push_back(p);

                //PatchVisualizer::visualize(geode, *it);
            }
        }
    }

// %EndTag(HELIX)%

    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
}
// %EndTag(SLEEP_END)%
    return 0;
}
