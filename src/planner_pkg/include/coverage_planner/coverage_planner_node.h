#ifndef _COVERAGE_NODE_H_
#define _COVERAGE_NODE_H_

#include <ros/ros.h>


#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseStamped.h>


#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include "../../include/coverage_planner/coverage_planner.h"

#include "simple_mapf_sim/PoseStampedArray.h"
#include "simple_mapf_sim/BatteryArray.h"
#include "simple_mapf_sim/TargetPoses.h"
#include "simple_mapf_sim/ObstacleArray.h"

#include <string>
#include <iostream> 
#include <algorithm>
#include <vector>
#include <stdlib.h>

class CoveragePlannerNode{
    public:
        CoveragePlannerNode()
        {
            _received_map = _received_obstacles = _received_init_pose = false;
            _published_count = 0;
            ros_rate = 1;

            planner = CoveragePlanner();
        }
        ~CoveragePlannerNode(){};
        void Run();

    private:

        // ROS Node Handlers
        ros::NodeHandle nh;
        
        // ROS Subscribers
        ros::Subscriber _occupancy_grid_sub;
        ros::Subscriber _obstacle_sub;
        ros::Subscriber _odom_sub;
    
        // ROS Publishers
        ros::Publisher plan_publisher;

        void OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg);
        void OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        void ObstacleHandler(const simple_mapf_sim::ObstacleArray::ConstPtr& msg);

        float ros_rate;
        geometry_msgs::PoseStamped robot_pose;
        
        CoveragePlanner planner;

        bool _received_map, _received_obstacles, _received_init_pose;
        int _published_count;
};
#endif
