#include "../../include/coverage_planner/coverage_planner_node.h"

void CoveragePlannerNode::Run()
{
    // Create ROS Subs
    _occupancy_grid_sub = nh.subscribe("/drone_sim/occupancy_grid", 1, &CoveragePlannerNode::OccupancyGridHandler, this);
    _obstacle_sub = nh.subscribe("/drone_sim/obstacles", 1, &CoveragePlannerNode::ObstacleHandler, this);
    _odom_sub = nh.subscribe("/drone_sim/vehicle_poses", 1, &CoveragePlannerNode::OdometryHandler, this);

    plan_publisher = nh.advertise<simple_mapf_sim::Plan>("/planning/global", 1, true);
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);
    while(ros::ok())
    {
        if (_received_map && _received_obstacles && _received_init_pose)
        {
            simple_mapf_sim::Plan plan;
            plan.vehicle_id = 0;
            plan.header.frame_id = "local_enu";
            plan.header.stamp = ros::Time::now();
            // int planLength = planner.naiveLawnmowerPlan(plan);
            int planLength = planner.plan(plan);
            if (plan.plan.size() > 0 && (_published_count < 3))
            {
                ROS_INFO("Publishing plan:");
                plan_publisher.publish(plan);
                _published_count++;
                int i = 1;
                for (const auto& wp : plan.plan)
                {
                    // ROS_INFO("    Waypoint %d: (%f,%f,%f)",i,wp.position.position.x,wp.position.position.y,wp.position.position.z);
                    i++;
                }
            }
            _received_map = _received_obstacles = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void CoveragePlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (!_received_map)
    {
        planner.updateMap(msg);
        _received_map = true;
    }
}

void CoveragePlannerNode::ObstacleHandler(const simple_mapf_sim::ObstacleArrayConstPtr& msg)
{
    if (!_received_obstacles)
    {
        planner.updateObstacles(msg);
        _received_obstacles = true;
    }
}

void CoveragePlannerNode::OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg)
{
    if (!_received_init_pose)
    {
        planner.setRobotLocation(msg->poses[0]);
        _received_init_pose = true;
    }
}

int main(int argc, char** argv)
{
    ROS_INFO("Starting Global Planner Node");
    ros::init(argc, argv, "GlobalPlannerNode");
    CoveragePlannerNode node;
    node.Run();
    return 0;
}