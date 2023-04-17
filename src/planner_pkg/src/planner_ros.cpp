#include "planner_pkg/planner_ros.h"

PlannerNode::PlannerNode(){

    initialized_map = false;
    init_robots_pose = true;
    init_targets = false;
    compute_plan = false;
    planner_type = 0;

    ros_rate = 1;
}


PlannerNode::~PlannerNode(){}


void PlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/sim/occupancy_grid", 1, &PlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/sim/vehicle_poses", 1, &PlannerNode::OdometryHandler, this);
    // target_locations_sub = nh.subscribe("/drone_sim/target_poses", 1, &PlannerNode::TargetHandler, this);

    compute_plan_sub = nh.subscribe("/planner/start", 1, &PlannerNode::ComputePlanHandler, this);

    plan_publisher = nh.advertise<simple_mapf_sim::MultiRobotPlan>("/planner/paths", 1, true);
    stats_pub = nh.advertise<planner_pkg::PlannerStats>("/planner/stats", 1);
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){
        if(initialized_map && init_targets && init_robots_pose && compute_plan){
            simple_mapf_sim::MultiRobotPlan plan;
            // plan.vehicle_id = 0;
            // plan.header.frame_id = "local_enu";
            // plan.header.stamp = ros::Time::now();
            

            initialized_map = false;
            init_robots_pose = false;
            compute_plan = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

}


void PlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    if(!initialized_map){
        // ROS_INFO("Updating map for planner");
        


        initialized_map = true;
    }
}

//  void PlannerNode::TargetHandler(const simple_mapf_sim::TargetPoses::ConstPtr& msg){
//     if(!init_targets){
//         // ROS_INFO("Updating targets for planner");
//         planner.setTargets(*msg);
//         init_targets = true;
//     }
// }

void PlannerNode::OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg){
    if(!init_robots_pose){
        // ROS_INFO("Updating robot pose for planner");
        
        
        init_robots_pose = true;
    }
}

void PlannerNode::ComputePlanHandler(const std_msgs::UInt8::ConstPtr& msg){
    if(!compute_plan){
        planner_type = msg->data;
        compute_plan = true;
    }
}

int main(int argc, char** argv) {

    ROS_INFO("Starting Landing Planner Node ");
    ros::init(argc, argv, "Landing Planner Node");
    PlannerNode node;
    node.Run();
    return 0;
}