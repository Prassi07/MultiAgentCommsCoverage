#ifndef _COVERAGE_PLANNER_H_
#define _COVERAGE_PLANNER_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "simple_mapf_sim/TargetPoses.h"
#include "simple_mapf_sim/TargetPose.h"
#include "simple_mapf_sim/Waypoint.h"
#include "simple_mapf_sim/Plan.h"
#include "simple_mapf_sim/ObstacleArray.h"
#include "simple_mapf_sim/ObstaclePose.h"

#include<numeric>
#include<queue>
#include<list>
#include<unordered_map>
#include <unordered_set>
#include<ros/ros.h>

struct Node
{
    double g = std::numeric_limits<double>::max();
    int idx = -1;
    int path_len = std::numeric_limits<int>::max();
    int parent_idx = -1;
 
    Node(){}
    Node(double g, int idx, int path_len, int parent) : g(g), idx(idx), path_len(path_len), parent_idx(parent) {}
    bool operator==(const Node& rhs) { return (idx = rhs.idx); }
};

class CompareGreaterLen
{
    public:
        bool operator() (const Node& lhs, const Node& rhs) const
        {
            return (lhs.path_len) > (rhs.path_len);
        }
};
class CompareGreaterCost
{
    public:
        bool operator() (const Node& lhs, const Node& rhs) const
        {
            return lhs.g > rhs.g;
        }
};
using OpenLenQueue = std::priority_queue<Node, std::vector<Node>, CompareGreaterLen>;
using OpenCostQueue = std::priority_queue<Node, std::vector<Node>, CompareGreaterCost>;

//Defining a custom closed list of type unordered_map
typedef std::unordered_map<int, Node*> CLOSED_LIST;

class CoveragePlanner{
    public:
        CoveragePlanner(int size_x, int size_y, int x_offset, int y_offset) : 
            _xs(size_x),
            _ys(size_y), 
            _xo(x_offset), 
            _yo(y_offset)
        {};

        CoveragePlanner(){};
        ~CoveragePlanner(){};

        int plan(simple_mapf_sim::Plan& plan);
        int naiveLawnmowerPlan(simple_mapf_sim::Plan& plan);
        
        void updateMap(const nav_msgs::OccupancyGrid::ConstPtr&);
        void updateObstacles(const simple_mapf_sim::ObstacleArrayConstPtr& obstacles);
        void setRobotLocation(const geometry_msgs::PoseStamped&);

        void printInfo();

    private:
        int _xs, _ys, _xo, _yo;
        float _map_resolution;
        int _map_size;

        int _coverage_size = 50;
        int _edge_buf = _coverage_size/2 - 2;
        
        float obstacle_cost;

        int _xinit, _yinit;
        int _path_height;
        
        // spiral ordering
        std::vector<std::pair<int,int>> _deltas = 
        {
            {-1,-1},
            {-1, 0},
            {-1, 1},
            { 0,-1},
            { 0, 1},
            { 1,-1},
            { 1, 0},
            { 1, 1},
        };

        std::vector<int8_t, std::allocator<int8_t>> _map = {};
        std::vector<uint8_t> _covered = {};
        std::vector<simple_mapf_sim::ObstaclePose> _obstacles = {};

        inline int xy2idx(int x, int y)
        {
            return (y*_ys + x);
        }

        inline std::pair<int,int> idx2xy(int mapidx)
        {
            return std::make_pair((mapidx % _xs), mapidx/_xs);
        }

        inline int numUnvisitedCells()
        {
            return _covered.size() - std::accumulate(_covered.begin(),_covered.end(),0) - 8000;
        }

        std::vector<int> generateNeighbors(int x, int y)
        {
            int dx, dy, newx, newy;
            std::vector<int> neighbors = {};
            for (const auto& delta : _deltas)
            {
                std::tie(dx,dy) = delta;
                newx = x+dx;
                newy = y+dy;
                // ROS_INFO("occ_map is %d at %d,%d",_map[xy2idx(newx,newy)],newx,newy);
                if ((newx > _edge_buf && newx < _xs-_edge_buf) &&
                    (newy > _edge_buf && newy < _ys-_edge_buf) &&
                    (_map[xy2idx(newx,newy)] < 100))
                {
                    neighbors.push_back(xy2idx(newx, newy));
                }
            }
            // ROS_INFO("Found %ld valid neighbors for %d,%d",neighbors.size(),x,y);
            return neighbors;
        }
        inline double dist(int x1, int y1, int x2, int y2)
        {
            return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        }

        int dist(int idx1, int idx2)
        {
            int x1, y1, x2, y2;
            std::tie(x1,y1) = idx2xy(idx1);
            std::tie(x2,y2) = idx2xy(idx2);

            return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        }

        void planToNearestUnvisited(int x, int y, std::list<simple_mapf_sim::Waypoint>& segment);

        void updateCoverageFromPose(int x, int y);

        void updateCoverageFromSegment(const std::list<simple_mapf_sim::Waypoint>& segment);

};

#endif
