#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <tf/LinearMath/Transform.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <fstream>
// #include "planning_msgs/Path.h"

class TimeLogger
{
public:
    TimeLogger() = default;
    ~TimeLogger() = default;
    void start()
    {
        start_ = std::chrono::high_resolution_clock::now();
    }
    double duration()
    {
        auto end = std::chrono::high_resolution_clock::now();
        double esplase = std::chrono::duration<double, std::ratio<1, 1000>>(end - start_).count();
        return esplase;
    }

private:
    std::chrono::high_resolution_clock::time_point start_;
};

class CSVLogger
{
public:
    CSVLogger(const std::string &csv_dir, const std::string prefix_name = "");
    ~CSVLogger();
    bool start(const std::string &header);

    template <typename... Args>
    void log(Args... args)
    {
        if (log_file_.is_open())
        {
            log_file_ << getLogTime() << ",";
            ((log_file_ << std::forward<Args>(args) << ","), ...);
            log_file_ << "\n";
        }
    }
    void stop();

private:
    std::string getFileTime();
    std::string getLogTime();

private:
    std::string log_dir_;
    std::string prefix_name_;
    std::ofstream log_file_;
    std::chrono::high_resolution_clock::time_point start_;
};

struct GridPoint
{
    int x = 0; // col
    int y = 0; // row
    GridPoint() = default;
    GridPoint(const int &in_x, const int &in_y) : x(in_x), y(in_y) {}
};

struct Node2d : GridPoint
{
    double f_cost = INFINITY; // 总消耗
    double g_cost = INFINITY; // 起点到中间消耗
    double h_cost = INFINITY; // 中间到终点消耗 启发项
    Node2d *parent = nullptr;

    Node2d() = default;
    Node2d(const int &in_x, const int &in_y) : GridPoint(in_x, in_y) {};
    bool operator>(const Node2d &other) const
    {
        return f_cost > other.f_cost; // 代價越大，越往後
    }
    bool operator==(const Node2d &other)
    {
        return (this->x == other.x && this->y == other.y);
    }
};

struct Node3d : GridPoint
{
    int yaw = 0;
    double wx = 0.0;
    double wy = 0.0;
    double wyaw = 0.0;
    double steer = 0.0;
    int gear = 1;
    double cost = 0.0;
    std::vector<Node3d *> head_nodes; // 前驱节点
    Node3d *parent = nullptr;
    boost::heap::fibonacci_heap<Node3d>::handle_type handle; // 用于更新优先队列
    Node3d() = default;
    Node3d(const double &in_wx, const double &in_wy, const double &in_wyaw) : wx(in_wx), wy(in_wy), wyaw(in_wyaw) {}
    bool operator<(const Node3d &other) const
    {
        return cost > other.cost; // 代價越大，越往後
    }
    bool operator==(const Node3d &other)
    {
        return (this->x == other.x && this->y == other.y && this->yaw == other.yaw);
    }
};

typedef boost::heap::fibonacci_heap<Node3d> PriorQueue;

struct CarModel
{
    double width;
    double length;
    double base_to_front; // base到前保险杠距离
    double base_to_back;  // base到后保险杠距离
    double l_wb;          // 轴距
    double max_steer;     // 最大转向角
};
void worldToGrid(const nav_msgs::OccupancyGrid &grid_map, const double &wx, const double &wy, int &out_gx, int &out_gy);
void gridToWorld(const nav_msgs::OccupancyGrid &grid_map, const int &gx, const int &gy, double &out_wx, double &out_wy);
int calcIndexInMap2d(const int &map_width, const int &gx, const int &gy);

void quaternionToEule(const double &in_x, const double &in_y, const double &in_z, const double &in_w,
                      double &out_roll, double &out_pitch, double &out_yaw);
geometry_msgs::Vector3 quaternionToEule(const geometry_msgs::Quaternion &in_quat);
void euleToQuaternion(const double &in_roll, const double &in_pitch, const double &in_yaw,
                      double &out_x, double &out_y, double &out_z, double &out_w);

geometry_msgs::Quaternion euleToQuaternion(geometry_msgs::Vector3 &eule);

#endif
