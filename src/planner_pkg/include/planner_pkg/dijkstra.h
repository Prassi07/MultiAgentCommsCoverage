#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

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

typedef std::shared_ptr<Node> Nodeptr;

//Defining a custom closed list of type unordered_map
typedef std::unordered_map<int, Nodeptr> CLOSED_LIST;

//Defining an open list of type priority queue
typedef std::priority_queue<Nodeptr, std::vector<Nodeptr>, CompareFValues> OPEN_LIST;

class CostMapPlanner{
    public:
        CostMapPlanner(const nav_msgs::OccupancyGrid &grid);
        CostMapPlanner();
        ~CostMapPlanner();

        void computeCostMap();
        int getCost(int x, int y);

        int states_expanded;
    private:
        int x_size, y_size, x_offset, y_offset;
        float map_resolution;
        int max_steps;
        float obstacle_cost;

        bool goal_updated;
        int dX[4] = {-1, 0, 0, 1};
        int dY[4] = { 0,-1, 1, 0};

        Node start_node;

        std::vector<int8_t, std::allocator<int8_t>> map;
        CLOSED_LIST closed_list;

        int computeKey(int, int);
        int getMapIndex(int, int);
};