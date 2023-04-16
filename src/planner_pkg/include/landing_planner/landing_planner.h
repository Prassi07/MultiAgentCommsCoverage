#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include "simple_mapf_sim/TargetPoses.h"
#include "simple_mapf_sim/TargetPose.h"
#include "simple_mapf_sim/Waypoint.h"
#include "simple_mapf_sim/Plan.h"

#include<queue>
#include<unordered_map>
#include<ros/ros.h>
#include<cmath>


#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


struct Node{
    int key;
    int x, y;
    int time;

    float g, h, f;
    std::shared_ptr<Node> parent;

    Node(){}

    Node(int x, int y):x(x),y(y)
    {
    }

    //Equals Comparision operator for unordered map
    bool operator==(const Node &n) const {
            return n.x == x && n.y == y;
    }

};

// Less than operator for priority queue
struct CompareFValues {
    bool operator()(std::shared_ptr<Node>  n1, std::shared_ptr<Node>  n2)
    {
        return n1->f > n2->f;
    }
};

// struct CompareFValues {
//     bool operator()(std::shared_ptr<Node>  n1, std::shared_ptr<Node>  n2)
//     {
//         if(n1->f == n2->f)
//             return n1->time < n2->time;
//         else
//             return n1->f > n2->f;
//     }
// };

typedef std::shared_ptr<Node> Nodeptr;

//Defining a custom closed list of type unordered_map
typedef std::unordered_map<int, Nodeptr> CLOSED_LIST;

//Defining an open list of type priority queue
typedef std::priority_queue<Nodeptr, std::vector<Nodeptr>, CompareFValues> OPEN_LIST;

class LandingPlanner{
    public:
        LandingPlanner(int size_x, int size_y, int x_offset, int y_offset);
        LandingPlanner();
        ~LandingPlanner();

        int planToGoals(simple_mapf_sim::Plan& plan);
        int naivePlanner(simple_mapf_sim::Plan& plan);
        
        void updateMap(const nav_msgs::OccupancyGrid::ConstPtr&);

        void setTargets(const simple_mapf_sim::TargetPoses&);
        void setRobotLocation(const geometry_msgs::PoseStamped&);
        void setBattery(const float);

        void printInfo();

        int states_expanded;
    private:
        int x_size, y_size, x_offset, y_offset;
        float map_resolution;
        int max_steps;
        float obstacle_cost;
        float start_battery, time_remaining;
        float robot_z;
        
        bool goal_updated;
        int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

        Node start_node;
        std::vector<Node> goal_locations;

        std::vector<int8_t, std::allocator<int8_t>> map;

        int estimateOctileDistance(int curr_x, int curr_y, int goal_x, int goal_y);
        int computeKey(int, int);
        int getMapIndex(int, int);
        void updateGoalCells();
        void updateStart();
        bool reachedAnyGoal(std::shared_ptr<Node>);

};