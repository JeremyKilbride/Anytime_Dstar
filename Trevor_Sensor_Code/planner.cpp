/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "../include/planner.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <queue> // Required for std::priority_queue
#include <functional>
#include <utility>
#include <unordered_set>
#include <cmath>
#include <chrono>




#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};
struct RobotState
{
    //info needed to initialize state
    std::pair<int, int> xyloc;// (x, y) location of the state
    std::shared_ptr<RobotState> parent_state;


    //values that are calculated
    double g_val;
    double f_val;
    double transition_cost; //cost allocated to the cell on the map
    double h_val;

    RobotState(std::pair<int, int> xyloc_, int* robot_map,int x_size,int y_size, int robotposeX,int robotposeY, std::shared_ptr<RobotState> parent_state_ = nullptr)
        : xyloc(xyloc_), parent_state(parent_state_)
    {
        transition_cost = robot_map[GETMAPINDEX(xyloc.first, xyloc.second, x_size, y_size)];
        if (parent_state != nullptr)
        {
            g_val = (parent_state->g_val) + transition_cost;
        }
        else
        {
            g_val = 0.0;
        }

        h_val = (double)sqrt(2) * MIN(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) +
            MAX(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) -
            MIN((xyloc.first - robotposeX), (xyloc.second - robotposeY));
        f_val = (double)g_val + 2.0 * h_val;

    }

    bool operator>(const RobotState& other) const
    {
        return f_val > other.f_val;
    }


};
struct CompareRobotState
{
    bool operator()(std::shared_ptr<RobotState> lhs, std::shared_ptr<RobotState> rhs)
    {
        
        return lhs->f_val > rhs->f_val;
    }
};


void planner(
    int* sensing_data,//local map
    int SENSING_RANGE,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr)
{
    std::vector<std::pair<int, int>> Best_Path;
    int robot_moves = 0;
    int min_x_local = std::max(1, robotposeX - SENSING_RANGE);
    int max_x_local = std::min(x_size, robotposeX + SENSING_RANGE);
    int min_y_local = std::max(1, robotposeY - SENSING_RANGE);
    int max_y_local = std::min(y_size, robotposeY + SENSING_RANGE);

    int x_size_local = max_x_local - min_x_local + 1;
    int y_size_local = max_y_local - min_y_local + 1;

    int robotposeX_local = robotposeX - min_x_local + 1;
    int robotposeY_local = robotposeY - min_y_local + 1;

    
    static int* robot_map = new int[x_size * y_size];
    if (curr_time == 0) 
    {
        for (int i = 0; i < x_size * y_size; i++)
        {
            robot_map[i] = 1; // Default cost assuming traversable
        }
    }
    
    for (int i = 1; i <= x_size_local; ++i) 
    {
        for (int j = 1; j <= y_size_local; ++j) 
        {
            int global_x = min_x_local + i - 1;
            int global_y = min_y_local + j - 1;
            if (global_x >= 1 && global_x <= x_size && global_y >= 1 && global_y <= y_size)
            {
                int map_idx = GETMAPINDEX(global_x, global_y, x_size, y_size);
                int map_idx_local = GETMAPINDEX(i, j, x_size_local, y_size_local);
                robot_map[map_idx] = sensing_data[map_idx_local]; // Update robot's map with sensed data
            }
        }
    }
    

    int dX[NUMOFDIRS] = { -1, -1, -1,  0,  0,  1, 1, 1 };
    int dY[NUMOFDIRS] = { -1,  0,  1, -1,  1, -1, 0, 1 };
    std::pair<int, int>start_pos = std::make_pair(robotposeX, robotposeY);
    static std::vector<std::shared_ptr<RobotState>> AllGoalStates;
    if (curr_time == 0)
    {
        //Calculate all Goal States
        
        int closest_index;
        double min_h_val;
        int qty_gs = 0;

        for (int i = 0; i <= (target_steps - 1 - curr_time); i++)
        {
            int goalposeX_poss = target_traj[curr_time + i];
            int goalposeY_poss = target_traj[curr_time + i + target_steps];
            int min_steps_togoal = (int)MAX(abs(robotposeX - goalposeX_poss), abs(robotposeY - goalposeY_poss));
            int robot_movetime = i;

            if (robot_movetime >= min_steps_togoal)//if its possible to meet the target along its trajectory at time step 
            {

                std::shared_ptr<RobotState> new_GS = std::make_shared<RobotState>(std::make_pair(goalposeX_poss, goalposeY_poss), robot_map, x_size, y_size, robotposeX, robotposeY);
                AllGoalStates.push_back(new_GS);
                if (qty_gs == 0) {
                    closest_index = qty_gs;
                    min_h_val = new_GS->h_val;
                }
                else if (new_GS->h_val < min_h_val)
                {
                    min_h_val = new_GS->h_val;
                    closest_index = qty_gs;
                }
                qty_gs = qty_gs + 1;
            }

        }
    }
    
    std::unordered_set<std::pair<int, int>, pair_hash> closed_list;
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState> open_list;

        for (std::shared_ptr<RobotState> GS : AllGoalStates)
        {
            open_list.push(GS);
        }
       

    bool valid_path = false;
    while (open_list.empty() != true) {
        std::shared_ptr<RobotState> RS_expand = open_list.top();
        open_list.pop(); //removes state being expanded from open list

        if (closed_list.find(RS_expand->xyloc) != closed_list.end()) {

            continue;
        }

        if (RS_expand->xyloc == start_pos) {
            valid_path = true;
                
            while (RS_expand != nullptr) {
                Best_Path.push_back(RS_expand->xyloc);
                RS_expand = RS_expand->parent_state;
            }
            std::cout << "Valid Path Found" << std::endl;

            break;
        }

        // Add state being expanded to the closed list

        closed_list.insert(RS_expand->xyloc);

        
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = RS_expand->xyloc.first + dX[dir];
            int newy = RS_expand->xyloc.second + dY[dir];
            

            std::pair<int, int> neighbor_key = std::make_pair(newx, newy);

            // Skip if the successor is in the closed list
            if (closed_list.find(neighbor_key) != closed_list.end()) {
                continue;
            }
            std::shared_ptr<RobotState> RS_candidate = std::make_shared<RobotState>(std::make_pair(newx, newy), robot_map, x_size, y_size, robotposeX, robotposeY, RS_expand);

            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)//if on map
            {
                
                if ((RS_candidate->transition_cost >= 0) && (RS_candidate->transition_cost < collision_thresh))  //if not an obstacle
                {
                    
                    open_list.push(RS_candidate);
        
                }

            }

        }

    }
    //Reverse A star search complete or no path found
    robot_moves++;
    if (valid_path == false)
    {
        std::cout << "No Path Found" << std::endl;
        Best_Path.push_back(std::make_pair(robotposeX, robotposeY));
        Best_Path.push_back(std::make_pair(robotposeX, robotposeY));
        std::cout << "Current Robot Position: " <<robotposeX<<" , "<<robotposeY<< std::endl;
        
    }
   
    action_ptr[0] = Best_Path[robot_moves].first;
    action_ptr[1] = Best_Path[robot_moves].second;
            
   

    return;
    
}
