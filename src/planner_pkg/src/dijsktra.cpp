
#include "planner_pkg/dijkstra.h"

using namespace std;


inline int CostMapPlanner::computeKey(int x, int y){
    return (y*y_size + x);
}

inline int CostMapPlanner::getMapIndex(int x, int y){
    return (y*y_size + x);
}

CostMapPlanner::CostMapPlanner(const nav_msgs::OccupancyGrid &grid){
    obstacle_cost = 100;
    goal_updated = false;
    map_resolution = grid.info.resolution;
    y_size = grid.info.width;
    x_size = grid.info.height;
    x_offset = (grid.info.origin.position.x)/(map_resolution);
    y_offset = (grid.info.origin.position.y)/(map_resolution);

    map.clear();
    map = grid.data;
}

CostMapPlanner::~CostMapPlanner(){}
CostMapPlanner::CostMapPlanner(){}


int CostMapPlanner::getCost(int x, int y){
    
    if(closed_list.count(computeKey(x, y)) != 0){
        Nodeptr node = closed_list[computeKey(x, y)];
        return node->g;
    }
    else{
       return -1; 
    }
    
}

void CostMapPlanner::computeCostMap(){
    
    //Setup start node, closed and open lists.
    Node start_node;
    start_node.x = (1/map_resolution) - x_offset;
    start_node.y = (1/map_resolution) - y_offset;

    OPEN_LIST open_list;
    Nodeptr start = std::make_shared<Node>(start_node.x, start_node.y);
    start->key = computeKey(start_node.x, start_node.y);
    start->g = 0;
    start->h = 0;
    start->f = start->g;
    start->parent = NULL;
    start->time = max_steps;
    
    int current_iter = 0;
    
    open_list.push(start);
    // ROS_INFO("Setup A star");
    bool pathFound = false;
    while(!open_list.empty()){

        Nodeptr curr_node = open_list.top();
        open_list.pop();
        int curr_key = computeKey(curr_node->x, curr_node->y);

        // ROS_INFO("Here1");
        if(closed_list.count(curr_key) == 0 ){

            closed_list.insert({curr_key, curr_node});
            
            for(int dir = 0; dir < 8; dir++)
            {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];

                bool expand = false;
                if(closed_list.count(computeKey(newx, newy)) == 0)
                    expand = true;

                if(closed_list.count(computeKey(newx, newy)) == 0){
                    if (newx >= 0 && newx < x_size && newy >= 0 && newy < y_size)
                    {
                        if (((int)map[getMapIndex(newx, newy)] >= 0) && ((int)map[getMapIndex(newx,newy)] < obstacle_cost))  //if free
                        {   
                            int g_s_dash = curr_node->g + (int)map[getMapIndex(newx, newy)];
                            int h_s_dash = 0;
                            Nodeptr successor = std::make_shared<Node>(newx, newy);
                            successor->parent = curr_node;
                            successor->g = g_s_dash;
                            successor->h = h_s_dash;
                            successor->f = g_s_dash + h_s_dash;
                            open_list.push(successor);
                        }
                    }
                }
            }
        }
            
    }

    states_expanded = closed_list.size();

}

