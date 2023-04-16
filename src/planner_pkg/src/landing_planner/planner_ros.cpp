#include "../../include/landing_planner/planner_ros.h"

LandingPlannerNode::LandingPlannerNode(){

    initialized_map = false;
    init_battery = false;
    init_robot_pose = true;
    init_targets = false;
    global_plan_recv = false;
    waypoint_num_recv = false;
    waypoint_number = 0;

    ros_rate = 1;

    planner = LandingPlanner();
}


LandingPlannerNode::~LandingPlannerNode(){}


void LandingPlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/drone_sim/coverage_grid", 1, &LandingPlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/drone_sim/vehicle_poses", 1, &LandingPlannerNode::OdometryHandler, this);
    battery_sub = nh.subscribe("/drone_sim/vehicle_battery",1, &LandingPlannerNode::BatteryHandler, this);
    target_locations_sub = nh.subscribe("/drone_sim/target_poses", 1, &LandingPlannerNode::TargetHandler, this);
    global_plan_sub = nh.subscribe("/planning/global",1, &LandingPlannerNode::GlobalPlanHandler, this);
    waypoint_num_sub = nh.subscribe("/behavior_executive/waypoint_number", 1, &LandingPlannerNode::WaypointNumHandler, this);

    plan_publisher = nh.advertise<simple_mapf_sim::Plan>("/planning/landing_zones", 1, true);
    stats_pub = nh.advertise<planner_pkg::PlannerStats>("/planner/stats", 1);
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){
        if(init_battery && initialized_map && init_targets && waypoint_num_recv){
            simple_mapf_sim::Plan plan;
            plan.vehicle_id = 0;
            plan.header.frame_id = "local_enu";
            plan.header.stamp = ros::Time::now();
            ros::Time s1 = ros::Time::now();
            int planLength = planner.planToGoals(plan);
            // int planLength = planner.naivePlanner(plan);
            ros::Time s2 = ros::Time::now();
            ros::Duration d = s2 - s1;
            if(planLength > 0){
                float dt = d.toSec();
                planner_pkg::PlannerStats stat;
                stat.time_taken = dt;
                stat.states_expanded = planner.states_expanded;
                ROS_INFO("Publishing plan of length: %d, Time taken to Plan: %f", planLength, dt);
                plan_publisher.publish(plan);
                stats_pub.publish(stat);
            }
            init_battery = false;
            initialized_map = false;
            // init_robot_pose = false;
            waypoint_num_recv = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

}

void LandingPlannerNode::BatteryHandler(const simple_mapf_sim::BatteryArray::ConstPtr& msg){

    if(!init_battery){
        // ROS_INFO("Updating battery for planner");
        float battery = msg->vehicles[0].percent;   
        planner.setBattery(battery); 
        init_battery = true;
    }
}

void LandingPlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    if(!initialized_map){
        // ROS_INFO("Updating map for planner");
        planner.updateMap(msg);
        initialized_map = true;
    }
}

 void LandingPlannerNode::TargetHandler(const simple_mapf_sim::TargetPoses::ConstPtr& msg){
    if(!init_targets){
        // ROS_INFO("Updating targets for planner");
        planner.setTargets(*msg);
        init_targets = true;
    }
}

void LandingPlannerNode::OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg){
    if(!init_robot_pose){
        // ROS_INFO("Updating robot pose for planner");
        planner.setRobotLocation(msg->poses[0]);
        init_robot_pose = true;
    }
}

void LandingPlannerNode::GlobalPlanHandler(const simple_mapf_sim::Plan::ConstPtr& msg){
    if(!global_plan_recv){
        ROS_INFO("Global Plan Received");
        global_plan = *msg;
        global_plan_recv = true;
    }
}

void LandingPlannerNode::WaypointNumHandler(const std_msgs::Float32 msg){
    if(global_plan_recv){
        if(!waypoint_num_recv){
            waypoint_number = msg.data;
            geometry_msgs::PoseStamped start;
            simple_mapf_sim::Waypoint wp = global_plan.plan[waypoint_number + 30];
            start.pose.position.x = wp.position.position.x;
            start.pose.position.y = wp.position.position.y;
            start.pose.position.z = wp.position.position.z;
            planner.setRobotLocation(start);
            waypoint_num_recv = true;
        }
    }
}

int main(int argc, char** argv) {

    ROS_INFO("Starting Landing Planner Node ");
    ros::init(argc, argv, "Landing Planner Node");
    LandingPlannerNode node;
    node.Run();
    return 0;
}