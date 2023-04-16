#ifndef _PLANNER_ROSN_H_
#define _PLANNER_ROS_H_

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "planner_pkg/PlannerStats.h"

#include "../../include/landing_planner/landing_planner.h"

#include "simple_mapf_sim/PoseStampedArray.h"
#include "simple_mapf_sim/BatteryArray.h"
#include "simple_mapf_sim/TargetPoses.h"

#include <string>
#include <iostream> 
#include <algorithm>
#include <vector>
#include <stdlib.h>

class LandingPlannerNode{
    public:
        LandingPlannerNode();
        ~LandingPlannerNode();
        void Run();

    private:

        // ROS Node Handlers
        ros::NodeHandle nh;
        ros::NodeHandle p_nh = ros::NodeHandle("~");
        
        // ROS Subscribers
        ros::Subscriber occupancy_grid_sub;
        ros::Subscriber target_locations_sub;

        ros::Subscriber battery_sub;
        ros::Subscriber odom_sub;

        ros::Subscriber global_plan_sub;
        ros::Subscriber waypoint_num_sub;
        ros::Publisher stats_pub;

        // ROS Publishers
        ros::Publisher plan_publisher;

        void OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg);
        void OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void BatteryHandler(const simple_mapf_sim::BatteryArray::ConstPtr& msg);
        void TargetHandler(const simple_mapf_sim::TargetPoses::ConstPtr& msg);
        void GlobalPlanHandler(const  simple_mapf_sim::Plan::ConstPtr& msg);
        void WaypointNumHandler(const std_msgs::Float32 msg);

        float ros_rate;
        geometry_msgs::PoseStamped robot_pose;
        float robot_battery;
        
        simple_mapf_sim::Plan global_plan;
        int waypoint_number;

        LandingPlanner planner;

        bool initialized_map, init_robot_pose, init_targets, init_battery, global_plan_recv, waypoint_num_recv;

};
#endif
