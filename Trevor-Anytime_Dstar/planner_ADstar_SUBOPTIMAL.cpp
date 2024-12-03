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
    std::shared_ptr<RobotState> parent_state;
    // Values that are calculated
    double g_val;
    double f_val;
    double transition_cost; // Cost allocated to the cell on the map
    double h_val;
    double v_val;
    std::vector<std::shared_ptr<RobotState>> successors;
    RobotState(std::pair<int, int> xyloc_, int* robot_map, int x_size, int y_size, int robotposeX, int robotposeY, double epsilon, std::shared_ptr<RobotState> parent_state_ = nullptr)
        : xyloc(xyloc_), parent_state(parent_state_) {
        transition_cost = robot_map[GETMAPINDEX(xyloc.first, xyloc.second, x_size, y_size)];
        if (parent_state != nullptr) {
            g_val = (parent_state->g_val) + transition_cost;
        }
        else {
            g_val = 0.0;
        }

        h_val = sqrt(2.0) * MIN(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) +
            MAX(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) -
            MIN(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY));
        //h_val= MAX(abs(robotposeX - xyloc.first), abs(robotposeY - xyloc.second));
        //h_val = sqrt(pow(xyloc.first - robotposeX, 2) + pow(xyloc.second - robotposeY, 2));

        update_f_val(epsilon);
        v_val = std::numeric_limits<double>::infinity();
    }

    void update_f_val(double epsilon) 
    {
        f_val = g_val + epsilon * h_val;
    }
    void update_h_val(int robotposeX, int robotposeY, double epsilon)
    {
        h_val = sqrt(2.0) * MIN(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) +
            MAX(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY)) -
            MIN(abs(xyloc.first - robotposeX), abs(xyloc.second - robotposeY));
        //h_val = sqrt(pow(xyloc.first - robotposeX, 2) + pow(xyloc.second - robotposeY, 2));
        update_f_val(epsilon);
    }
    void update_trans_cost(double tc, double epsilon)
    {
        transition_cost = tc;
        g_val= (parent_state->g_val) + transition_cost;
        update_f_val(epsilon);

    }
    void update_g_val(double new_gval, double epsilon)
    {
        g_val = new_gval;
        update_f_val(epsilon);
    }

    bool operator>(const RobotState& other) const {
        return f_val > other.f_val;
    }
};

struct CompareRobotState {
    bool operator()(const std::shared_ptr<RobotState>& lhs, const std::shared_ptr<RobotState>& rhs) const 
    {
        return lhs->f_val > rhs->f_val;
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
            std::shared_ptr<RobotState> new_GS = std::make_shared<RobotState>(std::make_pair(goalposeX_poss, goalposeY_poss), robot_map, x_size, y_size, robotposeX, robotposeY,epsilon);
            goal_found = true;
            return new_GS;
        }
    }
    if (goal_found == false) {
        return nullptr;
    }
}

//void Compute_Initial_Path(
//    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState>& open_list ,
//    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
//    int* robot_map,
//    const int collision_thresh,
//    const int x_size,
//    const int y_size,
//    const int robotposeX,
//    const int robotposeY,
//    const int curr_time,
//    std::vector<std::pair<int, int>>& Best_Path,
//    const std::shared_ptr<RobotState> GoalState,
//    std::shared_ptr<RobotState>&StartState,
//    const double epsilon
//    
//)
//{
//    std::cout << "Computing Initial Path " << std::endl;
//    int states_expanded = 0;
//    std::pair<int, int> start_pos = std::make_pair(robotposeX, robotposeY);
//    open_list.push(GoalState);
//    
//    bool valid_path = false;
//    while (!open_list.empty()) 
//    {
//        std::shared_ptr<RobotState> RS_expand = open_list.top();
//        open_list.pop();
//
//        auto it = closed_list.find(RS_expand->xyloc);
//        if (it != closed_list.end()) {
//            
//            continue;
//        }
//
//        if (RS_expand->xyloc == start_pos) {
//            valid_path = true;
//            StartState = RS_expand;
//            while (RS_expand != nullptr) {
//                Best_Path.push_back(RS_expand->xyloc);
//                RS_expand = RS_expand->parent_state;
//            }
//            std::cout << "Initial Path Constructed, States Expanded: "<<states_expanded <<", Epsilon= "<< epsilon<< std::endl;
//            std::cout << "Number of States left in open_list: " << open_list.size() << std::endl;
//            
//            break;
//        }
//
//        // Add state being expanded to the closed list
//        for (int dir = 0; dir < NUMOFDIRS; dir++) {
//            int newx = RS_expand->xyloc.first + dX[dir];
//            int newy = RS_expand->xyloc.second + dY[dir];
//            std::pair<int, int> neighbor_key = std::make_pair(newx, newy);
//
//            // Skip if the successor is in the closed list
//            if (closed_list.find(neighbor_key) != closed_list.end()) {
//                continue;
//            }
//            std::shared_ptr<RobotState> RS_candidate = std::make_shared<RobotState>(std::make_pair(newx, newy), robot_map, x_size, y_size, robotposeX, robotposeY,epsilon, RS_expand);
//
//            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) { // If on map
//                if ((RS_candidate->transition_cost >= 0) && (RS_candidate->transition_cost < collision_thresh)) 
//                { // If not an obstacle
//                    RS_expand->successors.push_back(RS_candidate);
//                    open_list.push(RS_candidate);
//                }
//            }
//        }
//        RS_expand->v_val = RS_expand->g_val;
//        states_expanded++;
//        closed_list[RS_expand->xyloc] = RS_expand;
//    }
//    // Reverse A* search complete or no path found
//    if (valid_path == false) {
//        std::cout << "No Path Found" << std::endl;
//        Best_Path.push_back(std::make_pair(robotposeX, robotposeY));
//        Best_Path.push_back(std::make_pair(robotposeX, robotposeY));
//        std::cout << "Current Robot Position: " << robotposeX << " , " << robotposeY << std::endl;
//    }
//}

void update_fvals(std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState> &open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    const double new_epsilon)
{
    
    for (auto& pair : closed_list)
    {
        pair.second->update_f_val(new_epsilon); 
    }

    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState> new_open_list;
    while (!open_list.empty())
    {
        std::shared_ptr<RobotState> RS = open_list.top();
        open_list.pop();
        RS->update_f_val(new_epsilon);
        new_open_list.push(RS);
        
    }
    open_list = new_open_list;
}

void Improve_Path_with_Reuse(
    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>&INCONS_list,
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
        StartState= std::make_shared<RobotState>(start_pos, robot_map, x_size, y_size, robotposeX, robotposeY, all_epsilons[0]);
        StartState->update_g_val(std::numeric_limits<double>::infinity(),all_epsilons[0]);

    }
    else
    {
        std::cout << "Improving Path, Robot Location: ( " <<robotposeX<<" , "<<robotposeY<<" )"<< std::endl;
        std::cout << "Num of Incons States: " <<INCONS_list.size() << std::endl;
        std::cout << "Num of States in Open: " << open_list.size() << std::endl;

        auto it = closed_list.find(std::make_pair(robotposeX, robotposeY));
        if (it != closed_list.end())
        {
            StartState = it->second;
        }
        else
        {
            std::cout << "Start State not found in closed list" << std::endl;
            StartState = std::make_shared<RobotState>(start_pos, robot_map, x_size, y_size, robotposeX, robotposeY, all_epsilons[0]);
            StartState->update_g_val(std::numeric_limits<double>::infinity(), all_epsilons[0]);
            open_list.push(StartState);
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
        
        std::vector<std::shared_ptr<RobotState>>INCONS_list_temp;
        int states_expanded = 0;
        std::vector<std::pair<int, int>>New_Path;
        auto planning_current_time = std::chrono::high_resolution_clock::now();
        auto planning_elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(planning_current_time - planning_start_time);
        if (planning_elapsed_time.count() >= planning_time_limit)
        {
            return;
        }
        std::cout << "Updating F vals" << std::endl;
        update_fvals(open_list, closed_list, epsilon);
        std::cout << "Loop If Statement" << std::endl;
        while (StartState->f_val> open_list.top()->f_val)
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
            if (RS_expand->xyloc == start_pos) 
            {
                std::cout << "Reached Start" << std::endl;
                RS_expand->v_val = RS_expand->g_val;
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
                        std::shared_ptr<RobotState> RS_candidate = std::make_shared<RobotState>(neighbor_key, robot_map, x_size, y_size, robotposeX, robotposeY, epsilon, RS_expand);
                        if (it != closed_list.end())
                        {
                            std::shared_ptr<RobotState>curr_state = it->second;

                            if (RS_candidate->f_val < curr_state->f_val)
                            {
                                
                                std::shared_ptr<RobotState>curr_state_parent = curr_state->parent_state;
                                curr_state_parent->successors.erase(
                                    std::remove(curr_state_parent->successors.begin(), curr_state_parent->successors.end(), curr_state),
                                    curr_state_parent->successors.end());

                                closed_list.erase(it);
                                INCONS_list_temp.push_back(RS_candidate);
                                RS_expand->successors.push_back(RS_candidate);
                       
                                continue;

                            }
                            else
                            {
                                continue;
                            }
                        }
                        open_list.push(RS_candidate);
                        RS_expand->successors.push_back(RS_candidate);

                    }

                }
            }
            RS_expand->v_val = RS_expand->g_val;
            states_expanded++;
            closed_list[RS_expand->xyloc] = RS_expand;

            if (open_list.empty())
            {
                std::cout << "Error: Open list is empty" << std::endl;
            }
        }
        std::cout << "Exited planning loop" << std::endl;
        std::shared_ptr<RobotState>nxt_state = StartState;
        bool path_found = false;
        while (nxt_state != nullptr) 
        {
            New_Path.push_back(nxt_state->xyloc);
            if (nxt_state->xyloc==GoalState->xyloc)
            {
                path_found = true;
            }
            nxt_state = nxt_state->parent_state;
           
        }
        if (path_found == false)
        {
            std::cout << "ERROR - Goal State not reached when back tracking " << std::endl;
        }
        Best_Path = New_Path;
        INCONS_list = INCONS_list_temp;
        std::cout << "Path Improved, States Expanded: " << states_expanded << ", Epsilon= " << epsilon << std::endl;
    }
}
struct SharedPtrHash {
    template <typename T>
    std::size_t operator()(const std::shared_ptr<T>& ptr) const {
        return std::hash<T*>()(ptr.get());
    }
};

// Custom equality function for shared_ptr
struct SharedPtrEqual {
    template <typename T>
    bool operator()(const std::shared_ptr<T>& lhs, const std::shared_ptr<T>& rhs) const {
        return lhs.get() == rhs.get();
    }
};

void propagateGValues(std::shared_ptr<RobotState> node, double epsilon, std::unordered_map<std::shared_ptr<RobotState>, bool, SharedPtrHash, SharedPtrEqual>& visited) {
    if (visited.count(node)) {
        return; 
    }
    visited[node] = true;

    for (auto& succ : node->successors) {
        
        succ->update_g_val(std::numeric_limits<double>::infinity(), epsilon);
        succ->v_val = std::numeric_limits<double>::infinity();

        // Recursively propagate to successors
        propagateGValues(succ, epsilon, visited);
    }
}

void apply_graph_updates(std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState>& open_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& closed_list,
    std::vector<std::shared_ptr<RobotState>>& INCONS_list,
    const double new_epsilon,
    std::vector<std::pair<int, int>> robot_map_changes,
    int* robot_map,
    const int collision_thresh,
    const int x_size,
    const int y_size,
    const int robotposeX,
    const int robotposeY
    )
    //remove undercons state from closed list if its above collision threshold. Remove It from parent's successor list
    //if below collision thresh, set v to inf, remove from closed and add to incons list
    //propogate g and v changes forward (set them to inf) 
{
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>new_closed_list;
    for (auto& pair : closed_list)
    {
        std::shared_ptr<RobotState>state_cl = pair.second;
        bool incons = false;
        for (std::pair<int, int> under_cons_loc : robot_map_changes)
        {
            if (state_cl->xyloc == under_cons_loc)
            {
                incons = true;
                int map_idx = GETMAPINDEX(state_cl->xyloc.first, state_cl->xyloc.second, x_size, y_size);
                double new_trans_cost = robot_map[map_idx];
                if (new_trans_cost >= collision_thresh)//is an obstacle
                {
                    new_trans_cost = std::numeric_limits<double>::infinity();
                    state_cl->update_trans_cost(new_trans_cost, new_epsilon);
                    state_cl->update_h_val(robotposeX, robotposeY, new_epsilon);
                    state_cl->v_val = std::numeric_limits<double>::infinity();
                    std::shared_ptr<RobotState>curr_state_parent = state_cl->parent_state;
                    curr_state_parent->successors.erase(
                        std::remove(curr_state_parent->successors.begin(), curr_state_parent->successors.end(), state_cl),
                        curr_state_parent->successors.end());

                    //dont keep in closed list
                    
                }
                else
                {
                    state_cl->update_trans_cost(new_trans_cost, new_epsilon);
                    state_cl->update_h_val(robotposeX, robotposeY, new_epsilon);
                    state_cl->v_val = std::numeric_limits<double>::infinity();
                    //dont keep in closed list
                    //add to Incons list
                    INCONS_list.push_back(state_cl);
                }
                //need to update all successors
                std::unordered_map<std::shared_ptr<RobotState>, bool, SharedPtrHash, SharedPtrEqual> visited;
                propagateGValues(state_cl, new_epsilon, visited);
                break;
            }
        }
        if (incons == false)
        {
            state_cl->update_h_val(robotposeX, robotposeY, new_epsilon);//updates h value and recalculates f with epsilon
            new_closed_list[state_cl->xyloc] = state_cl;
        }
        
    }
    closed_list = new_closed_list;

    std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState> new_open_list;
    while (!open_list.empty())
    {
        std::shared_ptr<RobotState> RS = open_list.top();
        open_list.pop();
        bool incons = false;
        for (std::pair<int, int> under_cons_loc : robot_map_changes)
        {
            if (RS->xyloc == under_cons_loc)
            {
                incons = true;
                int map_idx = GETMAPINDEX(RS->xyloc.first, RS->xyloc.second, x_size, y_size);
                double new_trans_cost = robot_map[map_idx];
                if (new_trans_cost >= collision_thresh)//is an obstacle
                {
                    new_trans_cost = std::numeric_limits<double>::infinity();
                    RS->update_trans_cost(new_trans_cost, new_epsilon);
                    RS->update_h_val(robotposeX, robotposeY, new_epsilon);
                    RS->v_val = std::numeric_limits<double>::infinity();
                    std::shared_ptr<RobotState>curr_state_parent = RS->parent_state;
                    curr_state_parent->successors.erase(
                        std::remove(curr_state_parent->successors.begin(), curr_state_parent->successors.end(), RS),
                        curr_state_parent->successors.end());
                }
                else
                {
                    RS->update_trans_cost(new_trans_cost, new_epsilon);
                    RS->update_h_val(robotposeX, robotposeY, new_epsilon);
                    RS->v_val = std::numeric_limits<double>::infinity();
                    new_open_list.push(RS);
                    
                }
                break;
            }
        }
        if (incons == false)
        {
            RS->update_h_val(robotposeX, robotposeY, new_epsilon);//updates h value and recalculates f with epsilon
            new_open_list.push(RS);
        }
    }
    open_list = new_open_list;
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
   
    std::vector<double> all_epsilons = {5.0, 2.0,1.5,1.2, 1.0 };
    double planning_time_limit = 990.0; //milliseconds
    auto planning_start_time = std::chrono::high_resolution_clock::now();

    std::vector<std::pair<int, int>> Best_Path;
    int robot_moves = 0;
    static int* robot_map = new int[x_size * y_size];
    
    static std::shared_ptr<RobotState> GoalState;

   
    std::vector<std::pair<int, int>> robot_map_changes = updaterobotmap_and_getchanges(robot_map, sensing_data, SENSING_RANGE, robotposeX,
        robotposeY, x_size, y_size, curr_time);

    static std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash> closed_list;
    static std::priority_queue<std::shared_ptr<RobotState>, std::vector<std::shared_ptr<RobotState>>, CompareRobotState> open_list;
    static std::vector<std::shared_ptr<RobotState>>INCONS_list;
    std::shared_ptr<RobotState> StartState=nullptr; //check if this needs to be static

    if (curr_time == 0)
    {
       Improve_Path_with_Reuse(open_list, closed_list,INCONS_list, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
                planning_time_limit, planning_start_time, Best_Path,GoalState,target_steps,target_traj,targetposeX,targetposeY);
    }
    else
    {
        apply_graph_updates(open_list, closed_list, INCONS_list, all_epsilons[0], robot_map_changes, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY);
        Improve_Path_with_Reuse(open_list, closed_list, INCONS_list, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
            planning_time_limit, planning_start_time, Best_Path, GoalState, target_steps, target_traj, targetposeX, targetposeY);

   
    }
    robot_moves++;
    action_ptr[0] = Best_Path[robot_moves].first;
    action_ptr[1] = Best_Path[robot_moves].second;
    

    return;
}
