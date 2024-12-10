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
const double INF = std::numeric_limits<double>::infinity();
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        auto h1 = std::hash<int>{}(p.first);
        auto h2 = std::hash<int>{}(p.second);
        return h1 ^ (h2 << 1); // Combines the two hashes
    }
};
struct RobotState {
    
    // Info needed to initialize state
    std::pair<int, int> xyloc; // (x, y) location of the state
    double rhs_val;
    double g_val;
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash> connections;
    

    RobotState(std::pair<int, int> xyloc_, double rhs_val_, double g_val_
        )
        : xyloc(xyloc_) ,rhs_val(rhs_val_),g_val(g_val_)
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

    if (RS->g_val > RS->rhs_val)
    {
        k1 = RS->rhs_val + epsilon * h_val;
        k2 =RS->rhs_val;
    }
    else
    {
        k1 = RS->g_val +  h_val;
        k2 = RS->g_val;
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




std::shared_ptr<RobotState>find_state(std::pair<int, int>loc,std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>states_in_set)
{
    auto it = states_in_set.find(loc);
    if (it == states_in_set.end())//not in set
    {
        return nullptr;
    }
    else
    {
        return it->second;
    }
}

bool isTimeLeft(const std::chrono::high_resolution_clock::time_point planning_start_time,double planning_time_limit)
{
    
    auto planning_current_time = std::chrono::high_resolution_clock::now();
    
    auto planning_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(planning_current_time - planning_start_time);
    
    if (planning_elapsed_time.count() >= planning_time_limit)
    {
        std::cout << "Time Elapsed" << std::endl;
        return false;
    }
    else
    {
        
        return true;
    }
}

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
            double GS_g_val = INF;
            std::shared_ptr<RobotState> new_GS = std::make_shared<RobotState>(std::make_pair(goalposeX_poss, goalposeY_poss),GS_rhs,GS_g_val );
            goal_found = true;
            return new_GS;
        }
    }
    if (goal_found == false) {
        std::cerr << "No goal found. Cannot start planning." << std::endl;
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

double get_transition_cost(const int* robot_map,const int collision_thresh, const std::pair<int, int>start_loc, const std::pair<int, int>end_loc, const int x_size,const int y_size)
{
    double tc= (double)robot_map[GETMAPINDEX(end_loc.first, end_loc.second, x_size, y_size)];
    return tc;
}

void remove_from_open(std::shared_ptr<RobotState>input_state, std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey>& open_list)
{
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>closed_map;
    while (!open_list.empty())
    {
        closed_map[open_list.top()->xyloc] = open_list.top();
        open_list.pop();
    }
    closed_map.erase(input_state->xyloc);
    for (auto &it : closed_map)
    {
        open_list.push(it.second);
    }

}

void connect_states(std::shared_ptr<RobotState>RS_A, std::shared_ptr<RobotState>RS_B)
{
   
    RS_A->connections[RS_B->xyloc] = RS_B;
    
    RS_B->connections[RS_A->xyloc] = RS_A;
    
}
void disconnect_states(std::shared_ptr<RobotState>RS_A, std::shared_ptr<RobotState>RS_B)
{

    RS_A->connections.erase(RS_B->xyloc);

    RS_B->connections.erase(RS_A->xyloc);

}
void UpdateState(std::shared_ptr<RobotState>input_state,
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareKey>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>& INCONS_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    std::shared_ptr<RobotState>& GoalState,
    const int collision_thresh,
    const int* robot_map,
    const int x_size,
    const int y_size
    )
{
    
    if (input_state->xyloc != GoalState->xyloc)
    {
        double min_rhs = INF;
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = input_state->xyloc.first + dX[dir];
            int newy = input_state->xyloc.second + dY[dir];
            std::pair<int, int> neighbor_loc = std::make_pair(newx, newy);
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                double transition_cost = get_transition_cost(robot_map, collision_thresh, input_state->xyloc, neighbor_loc, x_size, y_size);
                
                std::shared_ptr<RobotState> RS_succ = find_state(neighbor_loc, ALL_STATES);
                if (RS_succ == nullptr)
                {
                    RS_succ = std::make_shared<RobotState>(neighbor_loc, input_state->g_val + transition_cost,INF);
                    ALL_STATES[RS_succ->xyloc] = RS_succ;
                }
                connect_states(input_state, RS_succ);
                double new_rhs = transition_cost + RS_succ->g_val;
                if (new_rhs < min_rhs)
                {
                    min_rhs = new_rhs;
                }
                
            }

        }
        input_state->rhs_val = min_rhs;
        
    }
    if (open_list.empty() == false)
    {
        remove_from_open(input_state, open_list);
    }
    
    if (input_state->g_val != input_state->rhs_val)
    {
        if (find_state(input_state->xyloc, closed_list)==nullptr)//not in closed list
        {
            open_list.push(input_state);
        }
        else
        {
            INCONS_list.push_back(input_state);
        }

    }
    

}

bool isKeyLessThan(
    const std::pair<double, double>& keyA,
    const std::pair<double, double>& keyB)
{
    return (keyA.first < keyB.first) || (keyA.first == keyB.first && keyA.second < keyB.second);
}
void ComputeorImprovePath(
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
        StartState = std::make_shared<RobotState>(start_pos, INF, INF);
        ALL_STATES[GoalState->xyloc] = GoalState;
        ALL_STATES[StartState->xyloc] = StartState;
    }
    else
    {
        std::cout << "Improving Path, Robot Location: ( " <<robotposeX<<" , "<<robotposeY<<" )"<< std::endl;
        std::cout << "Num of States in Open: " << open_list.size() << std::endl;
        StartState = find_state(start_pos, ALL_STATES);
        
        if (StartState==nullptr)//This should never be the case
        {
            std::cout << "Start State Not Created Yet" << std::endl;
            StartState = std::make_shared<RobotState>(start_pos, INF, INF);
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
        
        if (isTimeLeft(planning_start_time, planning_time_limit) == false)
        {
            return;
        }
        INCONS_list.clear();
        closed_list.clear();
        std::vector<std::pair<int, int>>New_Path;

        Update_Epsilon_andResort(open_list, epsilon,robotposeX,robotposeY);
        
        
        while (!open_list.empty() &&
            (isKeyLessThan(getKey(open_list.top(), epsilon, robotposeX, robotposeY),
                getKey(StartState, epsilon, robotposeX, robotposeY))|| StartState->rhs_val != StartState->g_val))
        {
            
            
            if (isTimeLeft(planning_start_time, planning_time_limit) == false)
            {
                return;
            }

            std::shared_ptr<RobotState> RS_expand = open_list.top();
            open_list.pop();

            if (find_state(RS_expand->xyloc,closed_list)!=nullptr)
            {
                continue;
            }
          
            if (RS_expand->g_val > RS_expand->rhs_val)
            {
                RS_expand->g_val = RS_expand->rhs_val;
                closed_list[RS_expand->xyloc] = RS_expand;

                for (int dir = 0; dir < NUMOFDIRS; dir++)
                {

                    if (isTimeLeft(planning_start_time, planning_time_limit) == false)
                    {
                        return;
                    }

                    int newx = RS_expand->xyloc.first + dX[dir];
                    int newy = RS_expand->xyloc.second + dY[dir];

                    std::pair<int, int> neighbor_loc = std::make_pair(newx, newy);
                    if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
                    {
                        double transition_cost = get_transition_cost(robot_map, collision_thresh, RS_expand->xyloc, neighbor_loc, x_size, y_size);
                        
                        
                        std::shared_ptr<RobotState> RS_pred = find_state(neighbor_loc, ALL_STATES);
                        if (RS_pred == nullptr)
                        {
                            RS_pred = std::make_shared<RobotState>(neighbor_loc, RS_expand->g_val + transition_cost,INF);
                            ALL_STATES[RS_pred->xyloc] = RS_pred;
                        }
                        connect_states(RS_expand, RS_pred);

                        UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size);
                        

                    }

                }
                
            }
            else //if (RS_expand->g_val > RS_expand->rhs_val)//if underconsistent
            {
                RS_expand->g_val = INF;//make overconsistent
                for (int dir = 0; dir < NUMOFDIRS; dir++)
                {
                    if (isTimeLeft(planning_start_time, planning_time_limit) == false)
                    {
                        return;
                    }
                    int newx = RS_expand->xyloc.first + dX[dir];
                    int newy = RS_expand->xyloc.second + dY[dir];
                    std::pair<int, int> neighbor_loc = std::make_pair(newx, newy);
                    if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
                    {
                        double transition_cost = get_transition_cost(robot_map, collision_thresh, RS_expand->xyloc, neighbor_loc, x_size, y_size);
                        
                        std::shared_ptr<RobotState> RS_pred = find_state(neighbor_loc, ALL_STATES);
                        if (RS_pred == nullptr)
                        {
                            RS_pred = std::make_shared<RobotState>(neighbor_loc, INF, INF);
                            ALL_STATES[RS_pred->xyloc] = RS_pred;
                        }
                        connect_states(RS_expand, RS_pred);
                        UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size);
                        
                    }
                }
                
                UpdateState(RS_expand, open_list, closed_list, INCONS_list,ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size);

                
            }
            
            
        }
        std::cout << "---------------------Exited planning loop, epsilon ="<<epsilon<<"------------------- " << std::endl;

        std::shared_ptr<RobotState>current_state = StartState;
        New_Path.push_back(StartState->xyloc);
        bool path_found = false;
        std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>visited_states;
        visited_states[StartState->xyloc] = StartState;
        while (path_found==false) 
        {
            double min_cost= INF;
            std::shared_ptr<RobotState>best_succ;
            for (auto &it : current_state->connections)
            {
                std::shared_ptr<RobotState>succ = it.second;
                if (find_state(succ->xyloc, visited_states) != nullptr)
                {
                    continue;
                }
                if (get_transition_cost(robot_map, collision_thresh, current_state->xyloc, succ->xyloc, x_size, y_size) >= collision_thresh)
                {
                    continue;
                }
                double succ_cost = succ->g_val + get_transition_cost(robot_map,collision_thresh,current_state->xyloc,succ->xyloc, x_size, y_size);

                if (succ_cost < min_cost)
                {
                    best_succ = succ;
                    min_cost = succ_cost;
                }
            }
            if (best_succ==nullptr)
            {
                std::cout << "ERROR: no best successor found" << std::endl;
                break;
            }

            visited_states[best_succ->xyloc] = best_succ;
            New_Path.push_back(best_succ->xyloc);
            
            
            current_state = best_succ;
            if (current_state->xyloc == GoalState->xyloc)
            {
                path_found = true;
            }
        }
        if (path_found == true)
        {
            Best_Path = New_Path;
        }
        
        //std::cout << "Path Improved, States Expanded: " << states_expanded << ", Epsilon= " << epsilon << std::endl;
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
    
    for (std::pair<int, int>change_loc : robot_map_changes)
    {
       
        std::shared_ptr<RobotState>changed_state = find_state(change_loc, ALL_STATES);
        if (changed_state == nullptr)
        {
            changed_state = std::make_shared<RobotState>(change_loc, INF, INF);
        }
        
        for (int dir = 0; dir < NUMOFDIRS; dir++)
        {
            int newx = change_loc.first + dX[dir];
            int newy = change_loc.second + dY[dir];

            std::pair<int, int> neighbor_loc = std::make_pair(newx, newy);
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
            {
                double transition_cost = get_transition_cost(robot_map, collision_thresh, neighbor_loc,change_loc, x_size, y_size);
                std::shared_ptr<RobotState> RS_pred = find_state(neighbor_loc, ALL_STATES);
                
                if (RS_pred != nullptr)
                {
                    connect_states(RS_pred, changed_state);
                    UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size);
                }
                else
                {
                    RS_pred= std::make_shared<RobotState>(neighbor_loc, changed_state->g_val + transition_cost, INF);
                    ALL_STATES[RS_pred->xyloc] = RS_pred;
                    connect_states(RS_pred, changed_state);
                    UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size);
                }
            }

        }
        
       
    }
    
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
   
    const std::vector<double> all_epsilons = {50.0,5.0, 2.0,1.5,1.2, 1.0 };
    static double planning_time_limit;
    if (curr_time == 0)
    {
        std::cout << "Enter Planning Time Constraint (ms): ";
        std::cin >> planning_time_limit;
    }
    
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
       ComputeorImprovePath(open_list, closed_list,INCONS_list, ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
                planning_time_limit, planning_start_time, Best_Path,GoalState,target_steps,target_traj,targetposeX,targetposeY);
    }
    else
    {
        if (!robot_map_changes.empty())
        {
           std::cout << "Map Changes Detected" << std::endl;
           Update_States_w_Sensor_Info(open_list, closed_list, INCONS_list,GoalState,ALL_STATES, robot_map_changes, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY);
           std::cout << "States updated With Map Changes" << std::endl;
        }
        ComputeorImprovePath(open_list, closed_list, INCONS_list,ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
            planning_time_limit, planning_start_time, Best_Path, GoalState, target_steps, target_traj, targetposeX, targetposeY);

   
    }
    robot_moves++;
    action_ptr[0] = Best_Path[robot_moves].first;
    action_ptr[1] = Best_Path[robot_moves].second;
    

    return;
}
