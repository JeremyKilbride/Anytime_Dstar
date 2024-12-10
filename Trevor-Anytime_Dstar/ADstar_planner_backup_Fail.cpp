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
#include <unordered_map>
#include <limits>
#include <memory>
#include <algorithm>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
int dX[NUMOFDIRS] = { -1, -1, -1,  0,  0,  1, 1, 1 };
int dY[NUMOFDIRS] = { -1,  0,  1, -1,  1, -1, 0, 1 };

struct RobotState {
    
    // Info needed to initialize state
    std::pair<int, int> xyloc; // (x, y) location of the state
    double rhs_val;//current minimum cost to state
    double v_val;//cost after being expanded
    std::vector<std::shared_ptr<RobotState>> successors;
    std::vector<std::shared_ptr<RobotState>> predecessors;

    RobotState(std::pair<int, int> xyloc_, double rhs_val_, double v_val_
        )
        : xyloc(xyloc_) ,rhs_val(rhs_val_),v_val(v_val_)
    {
    }
};
double get_h_val(const std::shared_ptr<RobotState>&RS, const int robotposeX,const int robotposeY)
{
    double h_val = sqrt(2.0) * MIN(abs(RS->xyloc.first - robotposeX), abs(RS->xyloc.second - robotposeY)) +
        MAX(abs(RS->xyloc.first - robotposeX), abs(RS->xyloc.second - robotposeY)) -
        MIN(abs(RS->xyloc.first - robotposeX), abs(RS->xyloc.second - robotposeY));
    return h_val;
}

std::pair<double, double> getKey(const std::shared_ptr<RobotState>& RS, const double epsilon, const int robotposeX, const int robotposeY)
{
    double h_val = get_h_val(RS, robotposeX,robotposeY); 
    double k1;
    double k2;

    if (RS->v_val > RS->rhs_val)
    {
        k1 = RS->rhs_val + epsilon * h_val;
        k2 = RS->rhs_val;
    }
    else
    {
        k1 = RS->v_val +  h_val;
        k2 = RS->v_val;
    }
    
    return std::make_pair( k1, k2 );
}


struct CompareKey 
{
    int robotposeX;
    int robotposeY;
    double epsilon;
    CompareKey(int robotposeX_, int robotposeY_,double epsilon_) : robotposeX(robotposeX_), robotposeY(robotposeY_),epsilon(epsilon_) {}

    bool operator()(const std::shared_ptr<RobotState>& Astate, const std::shared_ptr<RobotState>& Bstate)
    {
        std::pair<double, double> keyA = getKey(Astate, epsilon, robotposeX, robotposeY);
        std::pair<double, double> keyB = getKey(Bstate, epsilon, robotposeX, robotposeY);
        return (keyA.first > keyB.first) || (keyA.first == keyB.first && keyA.second > keyB.second);
    }
};


struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        auto h1 = std::hash<int>{}(p.first);
        auto h2 = std::hash<int>{}(p.second);
        return h1 ^ (h2 << 1); // Combines the two hashes
    }
};

std::vector<std::pair<int, int>> updaterobotmap_and_getchanges(
    int* robot_map,
    int* sensing_data,
    const int SENSING_RANGE,
    const int robotposeX,
    const int robotposeY,
    const int x_size,
    const int y_size,
    const int curr_time)
{
    std::vector<std::pair<int, int>> cost_change_locations;
    int min_x_local = std::max(1, robotposeX - SENSING_RANGE);
    int max_x_local = std::min(x_size, robotposeX + SENSING_RANGE);
    int min_y_local = std::max(1, robotposeY - SENSING_RANGE);
    int max_y_local = std::min(y_size, robotposeY + SENSING_RANGE);

    int x_size_local = max_x_local - min_x_local + 1;
    int y_size_local = max_y_local - min_y_local + 1;

    int robotposeX_local = robotposeX - min_x_local + 1;
    int robotposeY_local = robotposeY - min_y_local + 1;

    if (curr_time == 0) {
        for (int i = 0; i < x_size * y_size; i++) {
            robot_map[i] = 1; // Default cost assuming traversable
        }
    }
    for (int i = 1; i <= x_size_local; ++i) {
        for (int j = 1; j <= y_size_local; ++j) {
            int global_x = min_x_local + i - 1;
            int global_y = min_y_local + j - 1;
            if (global_x >= 1 && global_x <= x_size && global_y >= 1 && global_y <= y_size) {
                int map_idx = GETMAPINDEX(global_x, global_y, x_size, y_size);
                int map_idx_local = GETMAPINDEX(i, j, x_size_local, y_size_local);
                if (robot_map[map_idx] != sensing_data[map_idx_local]) {
                    cost_change_locations.emplace_back(global_x, global_y);
                }
                robot_map[map_idx] = sensing_data[map_idx_local]; // Update robot's map with sensed data
            }
        }
    }
    return cost_change_locations;
}

std::shared_ptr<RobotState> Get_Goal_State(
    int* robot_map,
    const int collision_thresh,
    const int x_size,
    const int y_size,
    const int robotposeX,
    const int robotposeY,
    const int target_steps,
    int* target_traj,
    const int targetposeX,
    const int targetposeY,
    const int curr_time,
    const double epsilon)
{
    bool goal_found = false;
    for (int i = 0; i <= (target_steps - 1 - curr_time); i++) {
        int goalposeX_poss = target_traj[curr_time + i];
        int goalposeY_poss = target_traj[curr_time + i + target_steps];
        int min_steps_togoal = (int)MAX(abs(robotposeX - goalposeX_poss), abs(robotposeY - goalposeY_poss));
        int robot_movetime = i;

        if (robot_movetime >= min_steps_togoal) { // If it's possible to meet the target along its trajectory at time step
            double GS_rhs = 0.0;
            double GS_v_val = std::numeric_limits<double>::infinity();
            std::shared_ptr<RobotState> new_GS = std::make_shared<RobotState>(std::make_pair(goalposeX_poss, goalposeY_poss),GS_rhs,GS_v_val );
            goal_found = true;
            return new_GS;
        }
    }
    if (goal_found == false) {
        return nullptr;
    }
}



void Update_Epsilon_andResort(
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey> &open_list,
    const double new_epsilon,
    const int robotposeX,
    const int robotposeY)
{
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey> new_open_list(CompareKey(robotposeX,robotposeY, new_epsilon));
    while (!open_list.empty())
    {
        new_open_list.push(open_list.top());
        open_list.pop();
    }
    open_list = new_open_list;
}

void UpdateState(std::vector<std::shared_ptr<RobotState>>&input_states,
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>& INCONS_list,
    std::shared_ptr<RobotState>& GoalState,
    const int collision_thresh,
    const int* robot_map,
    const int x_size,
    const int y_size)
{
    for (std::shared_ptr<RobotState>state : input_states)
    {
        if (state->xyloc != GoalState->xyloc)
        {
            double min_rhs= std::numeric_limits<double>::infinity();

            for (std::shared_ptr<RobotState>succ : state->successors)
            {
                int transition_cost = robot_map[GETMAPINDEX(succ->xyloc.first, succ->xyloc.second, x_size, y_size)];
                if (transition_cost >= collision_thresh)
                {
                    transition_cost = std::numeric_limits<double>::infinity();
                }
                double new_rhs = transition_cost + succ->v_val;
                if (new_rhs < min_rhs)
                {
                    min_rhs = new_rhs;
                }
            }
            state->rhs_val = min_rhs;
        }
        if (state->v_val != state->rhs_val)
        {
            auto it = closed_list.find(state->xyloc);
            if (it == closed_list.end())//not in closed list
            {
                open_list.push(state);
            }
            else
            {
                INCONS_list.push_back(state);
            }

        }
    }

}

void Improve_Path_with_Reuse(
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>&INCONS_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    int* robot_map,
    const int collision_thresh,
    const int x_size,
    const int y_size,
    const int robotposeX,
    const int robotposeY,
    const int curr_time,
    std::shared_ptr<RobotState>&StartState,
    const std::vector<double> all_epsilons,
    const double planning_time_limit,
    const std::chrono::high_resolution_clock::time_point& planning_start_time,
    std::vector<std::pair<int, int>>& Best_Path,
    std::shared_ptr<RobotState> &GoalState,
    const int target_steps,
    int* target_traj,
    const int targetposeX,
    const int targetposeY
    )
{
    std::pair<int, int> start_pos = std::make_pair(robotposeX, robotposeY);
    if (curr_time == 0)
    {
        std::cout << "Creating Initial Path" << std::endl;
        GoalState = Get_Goal_State(robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY,
            target_steps, target_traj, targetposeX, targetposeY, curr_time, all_epsilons[0]);

        open_list.push(GoalState);
        StartState = std::make_shared<RobotState>(start_pos, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        ALL_STATES[GoalState->xyloc] = GoalState;
        ALL_STATES[StartState->xyloc] = StartState;

        
    }
    else
    {
        std::cout << "Improving Path, Robot Location: ( " <<robotposeX<<" , "<<robotposeY<<" )"<< std::endl;
        std::cout << "Num of Incons States: " <<INCONS_list.size() << std::endl;
        std::cout << "Num of States in Open: " << open_list.size() << std::endl;
        auto it = ALL_STATES.find(start_pos);
        if (it != closed_list.end())
        {
            StartState = it->second;
        }
        else
        {
            std::cout << "Start State Not Created Yet" << std::endl;
            StartState = std::make_shared<RobotState>(start_pos, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
            ALL_STATES[StartState->xyloc] = StartState;
        }
    }
    
    for (double epsilon : all_epsilons)
    {
        if (!INCONS_list.empty())
        {
            for (std::shared_ptr<RobotState>inc_state : INCONS_list)
            {
                open_list.push(inc_state);
            }
        }
        INCONS_list.clear();
        //std::vector<std::shared_ptr<RobotState>>INCONS_list_temp;
        int states_expanded = 0;
        std::vector<std::pair<int, int>>New_Path;
        auto planning_current_time = std::chrono::high_resolution_clock::now();
        auto planning_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(planning_current_time - planning_start_time);
        if (planning_elapsed_time.count() >= planning_time_limit)
        {
            return;
        }
        std::cout << "Update Epsilon and Resort" << std::endl;

        Update_Epsilon_andResort(open_list, epsilon,robotposeX,robotposeY);
        closed_list.clear();

        std::cout << "Loop If Statement" << std::endl;

        while (getKey(open_list.top(),epsilon,robotposeX,robotposeY) < getKey(StartState, epsilon, robotposeX, robotposeY) || StartState->rhs_val != StartState->v_val)
        {
            auto planning_current_time = std::chrono::high_resolution_clock::now();
            auto planning_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(planning_current_time - planning_start_time);
            if (planning_elapsed_time.count() >= planning_time_limit)
            {
                return;
            }

            std::shared_ptr<RobotState> RS_expand = open_list.top();
            open_list.pop();
            auto it = closed_list.find(RS_expand->xyloc);
            if (it != closed_list.end()) 
            {
                continue;
            }
            if (robot_map[GETMAPINDEX(RS_expand->xyloc.first, RS_expand->xyloc.second, x_size, y_size)] >= collision_thresh)
            {
                continue;
            }

            if (RS_expand->v_val > RS_expand->rhs_val)
            {
                RS_expand->v_val = RS_expand->rhs_val;
                if (RS_expand->xyloc == start_pos)
                {
                    std::cout << "Reached Start" << std::endl;
                    StartState = RS_expand;
                    closed_list[RS_expand->xyloc] = RS_expand;
                    break;
                }

                for (int dir = 0; dir < NUMOFDIRS; dir++)
                {
                    auto planning_current_time = std::chrono::high_resolution_clock::now();
                    auto planning_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(planning_current_time - planning_start_time);
                    if (planning_elapsed_time.count() >= planning_time_limit)
                    {
                        return;
                    }

                    int newx = RS_expand->xyloc.first + dX[dir];
                    int newy = RS_expand->xyloc.second + dY[dir];
                    if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
                    {
                        int transition_cost = robot_map[GETMAPINDEX(newx, newy, x_size, y_size)];
                        if (transition_cost >= 0 && transition_cost < collision_thresh)
                        {
                            std::pair<int, int> neighbor_key = std::make_pair(newx, newy);
                            auto it = closed_list.find(neighbor_key);
                            double new_rhs = RS_expand->v_val + transition_cost;
                            double new_v_val = std::numeric_limits<double>::infinity();
                            
                            if (it != closed_list.end())
                            {
                                std::shared_ptr<RobotState>closed_state = it->second;

                                if (new_rhs < closed_state->v_val)
                                {
                                    closed_state->rhs_val = new_rhs;
                                    closed_state->predecessors.push_back(RS_expand);
                                    RS_expand->successors.push_back(closed_state);
                                    INCONS_list.push_back(closed_state);
                                    
                                    continue;
                                }
                                else
                                {
                                    continue;
                                }
                            }
                            //not in closed list-> create new state
                            std::shared_ptr<RobotState> RS_succ = std::make_shared<RobotState>(neighbor_key, new_rhs, new_v_val);
                            ALL_STATES[RS_succ->xyloc] = RS_succ;
                            RS_succ->predecessors.push_back(RS_expand);
                            RS_expand->successors.push_back(RS_succ);
                            open_list.push(RS_succ);
                            

                        }

                    }
                }
                states_expanded++;
                closed_list[RS_expand->xyloc] = RS_expand;
                UpdateState(RS_expand->predecessors, open_list, closed_list, INCONS_list, GoalState,collision_thresh, robot_map, x_size, y_size);
            }
            else //if (RS_expand->v_val > RS_expand->rhs_val)//if underconsistent
            {
                RS_expand->v_val = std::numeric_limits<double>::infinity();//make overconsistent
                std::vector<std::shared_ptr<RobotState>> input_states = RS_expand->predecessors;
                input_states.insert(input_states.begin(), RS_expand);
                UpdateState(input_states, open_list, closed_list, INCONS_list, GoalState,collision_thresh, robot_map, x_size, y_size);

                
            }
            

            if (open_list.empty())
            {
                std::cout << "Error: Open list is empty" << std::endl;
            }
        }
        std::cout << "Exited planning loop" << std::endl;
        std::shared_ptr<RobotState>nxt_state = StartState;
        New_Path.push_back(StartState->xyloc);
        bool path_found = false;
        while (nxt_state->xyloc !=GoalState->xyloc) 
        {
            double min_f_val= std::numeric_limits<double>::infinity();
            std::shared_ptr<RobotState>best_pred;
            for (std::shared_ptr<RobotState>pred : nxt_state->predecessors)
            {
                double pred_f_val = pred->v_val + robot_map[GETMAPINDEX(pred->xyloc.first, pred->xyloc.second, x_size, y_size)];
                if (pred_f_val < min_f_val)
                {
                    best_pred = pred;
                    min_f_val = pred_f_val;
                }
            }
            New_Path.push_back(best_pred->xyloc);
            nxt_state = best_pred;
            if (nxt_state->xyloc == GoalState->xyloc)
            {
                path_found = true;
            }
        }
        if (path_found == false)
        {
            std::cout << "ERROR - Goal State not reached when back tracking " << std::endl;
        }
        Best_Path = New_Path;
        
        std::cout << "Path Improved, States Expanded: " << states_expanded << ", Epsilon= " << epsilon << std::endl;
    }
}


void Update_States_w_Sensor_Info(
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>& INCONS_list,
    std::shared_ptr<RobotState>& GoalState,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    std::vector<std::pair<int, int>> robot_map_changes,
    int* robot_map,
    const int collision_thresh,
    const int x_size,
    const int y_size,
    const int robotposeX,
    const int robotposeY
    )
    
{
    std::vector<std::shared_ptr<RobotState>>all_states_to_change;

    for (std::pair<int, int>change_loc : robot_map_changes)
    {
        auto it = ALL_STATES.find(change_loc);
        if (it != ALL_STATES.end())
        {
            it->second->v_val= std::numeric_limits<double>::infinity();

            for (std::shared_ptr<RobotState>pred : it->second->predecessors)
            {
                all_states_to_change.push_back(pred);
            }
            
        }
    }
    UpdateState(all_states_to_change, open_list, closed_list, INCONS_list, GoalState, collision_thresh, robot_map, x_size, y_size);
    
}



void planner(
    int* sensing_data, // Local map
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
   
    const std::vector<double> all_epsilons = {5.0, 2.0,1.5,1.2, 1.0 };
    double planning_time_limit = 990.0; //milliseconds
    auto planning_start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::pair<int, int>> Best_Path;
    int robot_moves = 0;
    static int* robot_map = new int[x_size * y_size];
    static std::shared_ptr<RobotState> GoalState;
    std::vector<std::pair<int, int>> robot_map_changes = updaterobotmap_and_getchanges(robot_map, sensing_data, SENSING_RANGE, robotposeX,
        robotposeY, x_size, y_size, curr_time);

    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash> closed_list;
    static std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash> ALL_STATES;

    static std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey> open_list(CompareKey( robotposeX, robotposeY,  all_epsilons[0]));

    static std::vector<std::shared_ptr<RobotState>>INCONS_list;
    std::shared_ptr<RobotState> StartState=nullptr;

    if (curr_time == 0)
    {
       Improve_Path_with_Reuse(open_list, closed_list,INCONS_list, ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
                planning_time_limit, planning_start_time, Best_Path,GoalState,target_steps,target_traj,targetposeX,targetposeY);
    }
    else
    {
        if (!robot_map_changes.empty())
        {
           Update_States_w_Sensor_Info(open_list, closed_list, INCONS_list,GoalState,ALL_STATES, robot_map_changes, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY);
        }
        Improve_Path_with_Reuse(open_list, closed_list, INCONS_list,ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
            planning_time_limit, planning_start_time, Best_Path, GoalState, target_steps, target_traj, targetposeX, targetposeY);

   
    }
    robot_moves++;
    action_ptr[0] = Best_Path[robot_moves].first;
    action_ptr[1] = Best_Path[robot_moves].second;
    

    return;
}
