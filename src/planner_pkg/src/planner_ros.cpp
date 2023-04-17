#include "planner_pkg/planner_ros.h"

PlannerNode::PlannerNode(){

    initialized_map = false;
    init_robots_pose = true;
    init_targets = false;
    compute_plan = false;
    planner_type = 0;

    ros_rate = 1;
    w = 1.1;
}

PlannerNode::~PlannerNode(){}

inline int PlannerNode::getMapIndex(int x, int y){
    return (y*dimy + x);
}

void PlannerNode::Run(){

    // Create ROS Subs
    occupancy_grid_sub = nh.subscribe("/sim/occupancy_grid", 1, &PlannerNode::OccupancyGridHandler, this);
    odom_sub = nh.subscribe("/sim/vehicle_poses", 1, &PlannerNode::OdometryHandler, this);
    target_locations_sub = nh.subscribe("/sim/target_poses", 1, &PlannerNode::TargetHandler, this);

    compute_plan_sub = nh.subscribe("/planner/start", 1, &PlannerNode::ComputePlanHandler, this);

    plan_publisher = nh.advertise<simple_mapf_sim::MultiRobotPlan>("/planner/paths", 1, true);
    stats_pub = nh.advertise<planner_pkg::PlannerStats>("/planner/stats", 1);
    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){
        if(initialized_map && init_targets && compute_plan){
            simple_mapf_sim::MultiRobotPlan full_plan;
            // plan.vehicle_id = 0;
            // plan.header.frame_id = "local_enu";
            // plan.header.stamp = ros::Time::now();
            
            if (planner_type == 1){ // ECBS Planner

                // Setup planner
                ECBS_Environment mapf(dimx, dimy, obstacles, goals, false); // disappearAtGoal = false
                mapf_lib::ECBS<State, Action, int, Conflict, Constraints, ECBS_Environment> ecbs(mapf, w);
                std::vector<mapf_lib::PlanResult<State, Action, int>> solution;

                ROS_INFO("Starting Planner");
                ROS_INFO_STREAM("St Si " << startStates.size() << " GS: " <<goals.size() <<  "Obs: " << obstacles.size());
                // Compute Solution
                bool success = ecbs.search(startStates, solution);

                if(success){
                    ROS_INFO("Successfully found paths to goals using ECBS");
                    full_plan.plans.clear();
                    for(size_t r = 0; r < solution.size(); ++r){
                        simple_mapf_sim::Plan plan;
                        for(const auto& state : solution[r].states){
                            simple_mapf_sim::Waypoint wp;
                            wp.x = state.first.x;
                            wp.y = state.first.y;
                            wp.t = state.second;
                            plan.plan.push_back(wp);
                        }
                        full_plan.plans.push_back(plan);
                    }

                    plan_publisher.publish(full_plan);
                }
                else{
                    ROS_ERROR("ECBS Could Not Find a Solution!");
                }
            }

            initialized_map = false;
            init_targets = false;
            compute_plan = false;
        }
        ros::spinOnce();
        rate.sleep();
    }

}

void PlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    if(!initialized_map){
        ROS_INFO("Updating map for planner");
        map_resolution = msg->info.resolution;
        dimx = msg->info.height;
        dimy = msg->info.width;
        x_offset = (msg->info.origin.position.x)/(map_resolution);
        y_offset = (msg->info.origin.position.y)/(map_resolution);
        map = msg->data;

        for(int i = 0; i < dimx; i++){
            for(int j = 0; j < dimy; j++){
                if(map[getMapIndex(i, j)] >= 80){
                    obstacles.insert(Location(i, j));
                }
            }
        }
        initialized_map = true;
    }
}

 void PlannerNode::TargetHandler(const simple_mapf_sim::CommsNodeArray::ConstPtr& msg){
    if(!init_targets){
        ROS_INFO("Updating targets for planner");
        goals.clear();
        for(const auto& node : msg->nodes){
            int x = (node.x/map_resolution) - x_offset;
            int y = (node.y/map_resolution) - y_offset;
            goals.emplace_back(Location(x, y));
        }
        init_targets = true;
    }
}

void PlannerNode::OdometryHandler(const simple_mapf_sim::PoseStampedArray::ConstPtr& msg){
    // ROS_INFO("Updating robot pose for planner");
    startStates.clear();
    for(const auto &robot : msg->poses){
        int x = (robot.pose.position.x/map_resolution) - x_offset;
        int y = (robot.pose.position.y/map_resolution) - y_offset;
        startStates.emplace_back(State(0, x, y));
    }
}

void PlannerNode::ComputePlanHandler(const std_msgs::UInt8::ConstPtr& msg){
    if(!compute_plan){
        ROS_INFO("Got command to start planner");
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