/**
 * @file  : simviewer.h
 * @brief : Viewer for simulator using ROS-RVIZ
 * @author: tigerxiang
 * @date  : 2022-02-11
 */

#ifndef _SIM_VIEWER_H_
#define _SIM_VIEWER_H_

#include <cstring>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Point.h"

#include "view/bypass_msg.h"    
#include "view/download_msg.h"    
#include "view/object.h"            
#include "view/platform_msg.h"       
#include "view/task_plan_status.h"
#include "view/can_comm_msg.h"  
#include "view/fault_msg.h"       
#include "view/path_plan_msg.h"     
#include "view/self_check_msg.h"     
#include "view/TLStatus.h"
#include "view/can_msg.h"       
#include "view/fms_msg.h"         
#include "view/path_plan_status.h"  
#include "view/sound_light_msg.h"    
#include "view/vehicle_task_status.h"
#include "view/control_msg.h"   
#include "view/hook_position.h"   
#include "view/perception.h"        
#include "view/state_machine_msg.h"
#include "view/decision_msg.h"  
#include "view/navigation_msg.h"  
#include "view/perception_msg.h"    
#include "view/task_plan_msg.h"
#include "view/ivmsglocpos.h"
#include "view/palletpos.h"

class ConvexPoint
{
public:
    ConvexPoint() {};
    ~ConvexPoint() {};

public:
    double x = 0.0;   //golobal x
    double y = 0.0;   //golobal y
    double z = 0.0;   //golobal z
    double xg = 0.0;  //local x
    double yg = 0.0;  //local y
    double zg = 0.0;  //local z
    double yaw = 0.0; 
    double pitch = 0.0;
    double roll = 0.0;
    double speed = 0.0;
};

class BoundBox
{
public:
    BoundBox() {};
    ~BoundBox() {};

public:
    int id = 0;
    int type = 0;         //0-car, 1-pedestrian, 2-cyclist, 3-unknown
    double x = 0.0;       //x position in map
    double y = 0.0;       //y position in map
    double dx = 0.0;      //length, size in x direction of car frame
    double dy = 0.0;      //width
    double heading = 0.0; //box rotation heading in map frame
    double height = 0.0;  //box height
    double vx = 0.0;      //velocity x in map frame
    double vy = 0.0;      //velocity y in map frame

    std::vector<geometry_msgs::Point> polygons; //polygons of objects
};

typedef visualization_msgs::Marker SHAPE;
typedef visualization_msgs::MarkerArray SHAPES;
typedef std::vector<ConvexPoint> ConvexPoints;
typedef std::vector<BoundBox> OBJS;

namespace robot  {

/**
 * "class SimViewer
 * "brief SimViewer is a class for simulator view.
 *        Based on ROS-RVIZ for visualization.
 *        HD map recommended.
 */
class SimViewer
{
public:
    SimViewer() {}
    SimViewer(double lat, double lon) {}
    ~SimViewer() {}

public:
    class DisplaySwitch {
    public:
        int lidar = 0;
        int vehicle = 0;
        int map = 0;
        int routing = 0;
        int planning = 0;
        int control = 0;
        int grid = 0;
	int tracking = 0;
    }sw;

    int index = 0;
    ConvexPoint gnss;
    double hook_xg = 0.0;
    double hook_yg = 0.0;
    double hook_heading = 0.0;

    /**
     * @brief : Set parameters for specify planner
     * @param : void
     * @return: Status
     */
    int SetParameters() {};

    /**
     * @brief : Draw vehicle in RVIZ
     * @param : Start id index for rviz, duration time
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawVehicle(int id, double t);

    /**
     * @brief : Draw lidar objects in RVIZ
     * @param : Object array, start id index for rviz, duration time
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawLidarObjects(const view::perception &objs, int id, double t);

    /**
     * @brief : Draw map in RVIZ
     * @param : Convex points, start id index for rviz, duration time
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawMap(const ConvexPoints &points, int id, double t);

    /**
     * @brief : Draw grid map in RVIZ
     * @param : Convex points, start id index for rviz, duration time
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawGrid(const ConvexPoints &points, int id, double t);

    /**
     * @brief : Draw dynamic tracking in RVIZ
     * @param : Convex points, start id index for rviz, duration time
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawTracking(const ConvexPoints &points, int id, double t);

    /**
     * @brief : Load map of format with lat/lon points
     * @param : Filename
     * @return: Convex points
     */
    ConvexPoints LoadMap(std::string file);

    /**
     * @brief : Draw path in RVIZ
     * @param : Convex points, duration time, RGB
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawPath(const ConvexPoints &points, double t, 
                    double r, double g, double b);

    /**
     * @brief : Draw point in RVIZ
     * @param : Convex point, duration time, RGB, scale xyz
     * @return: MakerArray for RVIZ
     */
    SHAPES DrawPoint(
           const ConvexPoint &point, int id, double t,
           double r, double g, double b,
           double x, double y, double z);
};

}  //namespace robot

/**
 * @brief : Callback rosmsg
 * @param : Ins message
 * @return: Void
 */
void GNSSCallback(const view::navigation_msg &msg);

/**
 * @brief : Callback rosmsg
 * @param : Perception message
 * @return: Void
 */
void PerceptionCallback(const view::perception &msg);

/**
 * @brief : Callback rosmsg
 * @param : Routing message
 * @return: Void
 */
void RoutingCallback(const view::path_plan_msg &msg);

/**
 * @brief : Callback rosmsg
 * @param : Planning message
 * @return: Void
 */
void PlanningCallback(const view::path_plan_msg &msg);

void PathStatusCallback(const view::path_plan_status &msg);

/**
 * @brief : Callback rosmsg
 * @param : Loadpos message
 * @return: Void
 */
void LoadposCallback(const view::palletpos &msg);

/**
 * @brief : ROS timer
 * @param : Void
 * @return: Void
 */
void Timer1Callback(const ros::TimerEvent &real);

extern robot::SimViewer *viewer;
extern ros::Publisher vehicle_pub;
extern ros::Publisher lidar_pub;
extern ros::Publisher map_pub;
extern ros::Publisher routing_pub;
extern ros::Publisher planning_pub;
extern ros::Publisher loadpos_pub;
extern ros::Publisher stoppose_pub;
extern ros::Publisher globalpath_sub;
extern tf::Transform odom_trans;

#endif  //_SIM_VIEWER_H_






