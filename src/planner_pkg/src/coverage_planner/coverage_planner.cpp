#include "../../include/coverage_planner/coverage_planner.h"
#include <chrono>

void CoveragePlanner::setRobotLocation(const geometry_msgs::PoseStamped& pose)
{
    _xinit = pose.pose.position.x;
    _yinit = pose.pose.position.y;
    ROS_INFO("Robot is starting at (%d, %d)", _xinit, _yinit);
}

void CoveragePlanner::updateMap(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    _map_resolution = grid->info.resolution;
    _ys = grid->info.width;
    _xs = grid->info.height;
    _xo = (grid->info.origin.position.x)/(_map_resolution);
    _yo = (grid->info.origin.position.y)/(_map_resolution);

    _map.clear();
    _map = grid->data;
    _map_size = _xs; // assuming square

    _covered.clear();
    _covered.resize(_map.size());
    ROS_INFO("xinit: %d, yinit: %d, xo: %d, yo: %d, num cell: %ld", _xinit, _yinit, _xo, _yo, _map.size());
}

void CoveragePlanner::updateObstacles(const simple_mapf_sim::ObstacleArrayConstPtr& obstacles)
{
    _obstacles = obstacles->obstacles;
    float temp_height = std::numeric_limits<float>::max();
    for (const auto& obs : _obstacles)
    {
        if (obs.height < temp_height) { temp_height = obs.height; }
    }
    _path_height = temp_height/2;
    // ROS_INFO("Received obstacles. There are %ld of them. I will fly at a height of %d", _obstacles.size(), _path_height);
}

// Update the coverage map for all cells visible from (x,y)
// x,y are in grid coords: (0,0) at corner, not center
void CoveragePlanner::updateCoverageFromPose(int x, int y)
{
    int start_x = (int) ((x - _coverage_size / 2.) / _map_resolution);
    int end_x = (int) ((x + _coverage_size / 2.) / _map_resolution);
    int start_y = (int) ((y - _coverage_size / 2.)  / _map_resolution);
    int end_y = (int) ((y + _coverage_size / 2.)  / _map_resolution);

    start_x = std::max(0, std::min(start_x, _xs));
    end_x = std::max(0, std::min(end_x, _xs));
    start_y = std::max(0, std::min(start_y, _ys));
    end_y = std::max(0, std::min(end_y, _ys));
    // ROS_INFO("    inside updateCov. bounds are %d < x < %d, %d < y < %d", start_x, end_x, start_y, end_y);
    for (int i = start_y; i < end_y; ++i)
    {
        for (int j = start_x; j < end_x; ++j)
        {
            _covered[xy2idx(j,i)] = 1;
        }
    }
}

// Update the coverage map for all cells visible along a sequence of waypoints
void CoveragePlanner::updateCoverageFromSegment(const std::list<simple_mapf_sim::Waypoint>& segment)
{
    for (const auto& wp : segment)
    {
        updateCoverageFromPose(wp.position.position.x, wp.position.position.y);
    }
}


// x,y are in grid coords
void CoveragePlanner::planToNearestUnvisited(int x, int y, std::list<simple_mapf_sim::Waypoint>& segment)
{
    int curidx = xy2idx(x,y);
    if (!_covered[curidx])
    {
        simple_mapf_sim::Waypoint wp;
        wp.position.position.x = x;
        wp.position.position.y = y;
        wp.position.position.z = _path_height;
        segment.push_back(wp);
        return;
    }

    std::unordered_set<int> closed_set = {};
    std::unordered_map<int,Node> costs = {};
    OpenCostQueue open_cost_queue = {};

    Node init_node(0,curidx,0,-1);
    open_cost_queue.push(init_node);
    costs[curidx] = init_node;
    Node s;
    int cx, cy;
    while (!open_cost_queue.empty())
    {
        s = open_cost_queue.top();
        open_cost_queue.pop();

        if (closed_set.count(s.idx)) { continue; }

        closed_set.insert(s.idx);

        // Found a cell that hasn't been seen yet
        if (!_covered[s.idx]) { break; }

        std::tie(cx,cy) = idx2xy(s.idx);
        for (auto& succ : generateNeighbors(cx,cy))
        {
            double new_cost = s.g + dist(s.idx,succ);
            if (costs[succ].g > new_cost)
            {
                Node successor_node{new_cost, succ, s.path_len+1, s.idx};

                costs[succ] = successor_node;
                open_cost_queue.push(successor_node);
            }
        }
    }
    // backtrack
    curidx = s.idx;
    std::vector<int> path{curidx};
    while (costs[curidx].parent_idx != -1)
    {
        path.push_back(costs[curidx].parent_idx);
        curidx = costs[curidx].parent_idx;
    }
    std::reverse(path.begin(),path.end());
    for (const auto& idx : path)
    {
        simple_mapf_sim::Waypoint wp;
        std::tie(wp.position.position.x, wp.position.position.y) = idx2xy(idx);
        wp.position.position.z = _path_height;
        segment.push_back(wp);
    }
}

int CoveragePlanner::plan(simple_mapf_sim::Plan& plan)
{
    // Make sure we're not adding to a nonempty plan
    plan.plan.clear();

    // convert to map grid coordinates
    _xinit = _xinit/_map_resolution - _xo;
    _yinit = _yinit/_map_resolution - _yo;

    int curx = _xinit;
    int cury = _yinit;

    // ROS_INFO("Calling updateCoverageFromPose with %d,%d",curx,cury);
    updateCoverageFromPose(curx, cury);
    // ROS_INFO("Left updateCoverageFromPose");

    ROS_INFO("About to enter the while loop in plan");
    std::list<simple_mapf_sim::Waypoint> temp_plan;
    int unvisited_cnt = numUnvisitedCells();
    while (unvisited_cnt > 0)
    {
        // ROS_INFO("    There are %d unvisited cells remaining", unvisited_cnt);
        std::list<simple_mapf_sim::Waypoint> segment;
        // ROS_INFO("    Planning from %d,%d to nearest unvisited cell", curx,cury);
        planToNearestUnvisited(curx, cury, segment);
        // ROS_INFO("    Found segment of length %ld to %f,%f", segment.size(), segment.back().position.position.x, segment.back().position.position.y);
        updateCoverageFromSegment(segment);

        // splicing lists is faster than inserting into vectors
        temp_plan.splice(temp_plan.end(), segment);
        curx = temp_plan.back().position.position.x;
        cury = temp_plan.back().position.position.y;
        unvisited_cnt = numUnvisitedCells();
    }

    // copy to the vector for publishing
    plan.plan.insert(plan.plan.end(),temp_plan.begin(), temp_plan.end());

    // Shift back into sim coordinates
    for (auto& wp : plan.plan)
    {
        wp.position.position.x += _xo;
        wp.position.position.y += _yo;
    }

    return 0;
}

int CoveragePlanner::naiveLawnmowerPlan(simple_mapf_sim::Plan& plan)
{
    plan.plan.clear();
    // go to map corner
    simple_mapf_sim::Waypoint wp;
    wp.position.position.x = _xo;
    wp.position.position.y = _yo;
    wp.position.position.z = _path_height;
    plan.plan.push_back(wp);

    for (int col = 0; col*_coverage_size < (_map_size - 60.0); col++)
    {
        simple_mapf_sim::Waypoint wp;
        if (col % 2 == 0)
        {
            wp.position.position.x = -_xo;
        }
        else{
            wp.position.position.x = _xo;
        }
        wp.position.position.y = _yo+col*_coverage_size;
        wp.position.position.z = _path_height;
        plan.plan.push_back(wp);

        wp.position.position.y += _coverage_size;

        plan.plan.push_back(wp);
    }
    return 0;
}