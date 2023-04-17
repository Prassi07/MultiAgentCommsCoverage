#ifndef _PLANNER_ROS_H_
#define _PLANNER_ROS_H_

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include "planner_pkg/PlannerStats.h"

#include "ecbs_planner.h"
#include "planner_commons.h"

#include "simple_mapf_sim/PoseStampedArray.h"
#include "simple_mapf_sim/Waypoint.h"
#include "simple_mapf_sim/Plan.h"
#include "simple_mapf_sim/MultiRobotPlan.h"

#include <string>
#include <iostream> 
#include <algorithm>
#include <vector>
#include <stdlib.h>

class PlannerNode{
    public:
        PlannerNode();
        ~PlannerNode();
        void Run();

    private:

        // ROS Node Handlers
        ros::NodeHandle nh;
        ros::NodeHandle p_nh = ros::NodeHandle("~");
        
        // ROS Subscribers
        ros::Subscriber occupancy_grid_sub;
        ros::Subscriber target_locations_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber compute_plan_sub;

        // ROS Publishers
        ros::Publisher stats_pub;
        ros::Publisher plan_publisher;

        // Callback Methods
        void OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg);
        void OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        // void TargetHandler(const simple_mapf_sim::TargetPoses::ConstPtr& msg);
        void ComputePlanHandler(const std_msgs::UInt8::ConstPtr& msg);

        // Rosnode variables
        float ros_rate;
        geometry_msgs::PoseStamped robot_pose;
        simple_mapf_sim::MultiRobotPlan global_plan;

        int planner_type;
        bool initialized_map, init_robots_pose, init_targets, compute_plan;

        //Planner Variables
        int dimx, dimy;
        std::unordered_set<Location> obstacles;
        std::vector<Location> goals;
        std::vector<State> startStates;
};
#endif
