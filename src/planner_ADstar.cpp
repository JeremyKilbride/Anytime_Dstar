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

std::pair<int, int> GetXYFromIndex(const int index,const int& XSIZE,const int& YSIZE)
{
    int Y = (index / XSIZE) + 1;
    int X = (index % XSIZE) + 1;
    return std::make_pair(X, Y);
}

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
    
    RobotState(std::pair<int, int> xyloc_, double rhs_val_, double g_val_
        )
        : xyloc(xyloc_) ,rhs_val(rhs_val_),g_val(g_val_)
    {
    }
};
struct OpenListEntry 
{
    std::shared_ptr<RobotState> state;
    std::pair<double, double> stored_key;
    OpenListEntry(std::shared_ptr<RobotState>& state_, std::pair<double, double>& stored_key_)
        : state(state_), stored_key(stored_key_)
    {
    }
};
double get_h_val(const std::shared_ptr<RobotState>&RS, const int &robotposeX,const int &robotposeY)
{
 
    double h_val = MAX(abs(RS->xyloc.first - robotposeX), abs(RS->xyloc.second - robotposeY)) ;
    return h_val;
}

double get_minsteps_toloc(const std::shared_ptr<RobotState>& RS, const int& X, const int& Y)
{
    double steps = MAX(abs(X - RS->xyloc.first), abs(Y - RS->xyloc.second));
    return steps;
}

std::pair<double, double> getKey(const std::shared_ptr<RobotState>& RS, const double& epsilon, const int& robotposeX, const int& robotposeY)
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
    bool operator()(const std::shared_ptr<OpenListEntry>& a, const std::shared_ptr<OpenListEntry>& b) const {
        return (a->stored_key.first > b->stored_key.first) ||
            (a->stored_key.first == b->stored_key.first && a->stored_key.second > b->stored_key.second);
    }
};

std::shared_ptr<RobotState>find_state(const std::pair<int, int>&loc,const std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>&states_in_set)
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
bool is_in_set(const std::pair<int, int>&loc,const std::unordered_set<std::pair<int, int>, PairHash> &locs_in_set)
{
    auto it = locs_in_set.find(loc);
    if (it == locs_in_set.end())//not in set
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool isTimeLeft(const std::chrono::high_resolution_clock::time_point &planning_start_time,const double& planning_time_limit)
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

std::vector<std::pair<int, int>> get_neighbors(const std::pair<int, int>& loc, const int& x_size,const int& y_size) 
{
    std::vector<std::pair<int, int>> neighbors;
    neighbors.reserve(8);
    for (int dir = 0; dir < NUMOFDIRS; dir++) {
        int newx = loc.first + dX[dir];
        int newy = loc.second + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size) {
            neighbors.emplace_back(newx, newy);
        }
    }
    return neighbors;
}

std::shared_ptr<RobotState> find_or_create_state(
    const std::pair<int, int>& loc,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& states_in_set)
{
    auto it = states_in_set.find(loc);
    if (it == states_in_set.end()) {
        auto new_state = std::make_shared<RobotState>(loc, INF, INF);
        states_in_set[loc] = new_state;
        return new_state;
    }
    return it->second;
}
std::vector<std::pair<int, int>> updaterobotmap_and_getchanges(
    int*& robot_map,
    int*& sensing_data,
    const int &SENSING_RANGE,
    const int &robotposeX,
    const int &robotposeY,
    const int &x_size,
    const int &y_size,
    const int &curr_time,
    const std::pair<int,int> &GoalLoc,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES
    )
{
    std::vector<std::pair<int, int>> cost_change_locations;
    cost_change_locations.reserve(x_size * y_size);
    int min_x_local = std::max(1, robotposeX - SENSING_RANGE);
    int max_x_local = std::min(x_size, robotposeX + SENSING_RANGE);
    int min_y_local = std::max(1, robotposeY - SENSING_RANGE);
    int max_y_local = std::min(y_size, robotposeY + SENSING_RANGE);

    int x_size_local = max_x_local - min_x_local + 1;
    int y_size_local = max_y_local - min_y_local + 1;

    int robotposeX_local = robotposeX - min_x_local + 1;
    int robotposeY_local = robotposeY - min_y_local + 1;

    if (curr_time == 0) 
    {
        for (int i = 0; i < x_size * y_size; i++) 
        {
            robot_map[i] = 1; 
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

void Update_Epsilon_andResort(
    std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey>&open_list,
    const double &new_epsilon,
    const int &robotposeX,
    const int &robotposeY)
{
    std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey> new_open_list;
    while (!open_list.empty())
    {
        std::shared_ptr<OpenListEntry>entry = open_list.top();
        if (entry->state == nullptr)
        {
            entry=nullptr;
            open_list.pop();
            continue;
        }
        entry->stored_key = getKey(entry->state, new_epsilon, robotposeX, robotposeY);
        new_open_list.push(entry);
        open_list.pop();
    }
    open_list = new_open_list;
}

double get_transition_cost(int*& robot_map,const int& collision_thresh, const std::pair<int, int>&end_loc, const int& x_size,const int& y_size)
{
    double tc= (double)robot_map[GETMAPINDEX(end_loc.first, end_loc.second, x_size, y_size)];
    return tc;
   
}

void UpdateState(std::shared_ptr<RobotState>&input_state,
    std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey>& open_list,
    std::unordered_set<std::pair<int, int>, PairHash>& closed_list,
    std::unordered_set<std::pair<int, int>, PairHash>& INCONS_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    std::shared_ptr<RobotState>& GoalState,
    const int &collision_thresh,
    int* &robot_map,
    const int &x_size,
    const int &y_size,
    const double& epsilon,
    const int &robotposeX,
    const int &robotposeY
    )
{
    
    if (input_state->xyloc != GoalState->xyloc)
    {
        
        double min_rhs = INF;
        std::vector<std::pair<int, int>>nbrs = get_neighbors(input_state->xyloc, x_size, y_size);
        for  (std::pair<int, int>succ_loc : nbrs)
        {    
            std::shared_ptr<RobotState> RS_succ = find_state(succ_loc, ALL_STATES);
            if (RS_succ == nullptr)
            {
                continue;
            }
            double transition_cost = get_transition_cost(robot_map, collision_thresh, succ_loc, x_size, y_size);
            double new_rhs = transition_cost + RS_succ->g_val;
            if (new_rhs < min_rhs)
            {
                min_rhs = new_rhs;
            }
        }
        input_state->rhs_val = min_rhs;
        
    }
   
   

    if (input_state->g_val != input_state->rhs_val)
    {
        
        if (is_in_set(input_state->xyloc, closed_list) == false)//not in closed list
        {
            std::shared_ptr<OpenListEntry> entry=std::make_shared<OpenListEntry>(input_state, getKey(input_state, epsilon, robotposeX, robotposeY));
            
            open_list.push(entry);
        }
        else
        {
            INCONS_list.emplace(input_state->xyloc.first, input_state->xyloc.second);
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
    std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey>& open_list,
    std::unordered_set<std::pair<int, int>, PairHash>& closed_list,
    std::unordered_set<std::pair<int, int>, PairHash>& INCONS_list,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    int* &robot_map,
    const int& collision_thresh,
    const int &x_size,
    const int &y_size,
    const int &robotposeX,
    const int &robotposeY,
    const int &curr_time,
    std::shared_ptr<RobotState>& StartState,
    const std::vector<double>& all_epsilons,
    const double &planning_time_limit,
    const std::chrono::high_resolution_clock::time_point& planning_start_time,
    std::vector<std::pair<int, int>>& Best_Path,
    std::shared_ptr<RobotState>& GoalState,
    const int &target_steps,
    int* &target_traj,
    const int& targetposeX,
    const int& targetposeY,
    int &curr_epsilon_idx,
    int &robot_moves
)
{
    std::pair<int, int> start_pos = std::make_pair(robotposeX, robotposeY);
    if (curr_time == 0)
    {
        std::cout << "Creating Initial Path" << std::endl;
    }
    else
    {
        std::cout << "Improving Path, Robot Location: ( " << robotposeX << " , " << robotposeY << " )" << std::endl;
        std::cout << "Num of States in Open: " << open_list.size() << std::endl;
    }
    int tot_n_states_expanded = 0;
    int i = curr_epsilon_idx;
    for (i; i < all_epsilons.size(); ++i)
    {
        double epsilon = all_epsilons[i];
        if (!INCONS_list.empty())
        {
            for (const auto& it : INCONS_list)
            {
                std::shared_ptr<RobotState>incons_state = find_state(it, ALL_STATES);
                if (incons_state == nullptr)
                {
                    continue;
                }

                std::shared_ptr<OpenListEntry>entry = std::make_shared<OpenListEntry>(incons_state, getKey(incons_state, epsilon, robotposeX, robotposeY));
                open_list.push(entry);
            }
        }
        

        if (isTimeLeft(planning_start_time, planning_time_limit) == false)
        {
            return;
        }
        INCONS_list.clear();
        closed_list.clear();
        std::vector<std::pair<int, int>>New_Path;
        New_Path.reserve(x_size * y_size);

        if (open_list.empty() == false)
        {
            Update_Epsilon_andResort(open_list, epsilon, robotposeX, robotposeY);
        }

        int n_states_expanded = 0;
        while(!open_list.empty() && open_list.top()==nullptr)
        {
            open_list.pop();
        }
        while(!open_list.empty() && open_list.top()->state==nullptr)
        {
            open_list.pop();
        }
        

        while (!open_list.empty() &&
            (isKeyLessThan(getKey(open_list.top()->state, epsilon, robotposeX, robotposeY),
                getKey(StartState, epsilon, robotposeX, robotposeY)) || StartState->rhs_val != StartState->g_val||( StartState->rhs_val==INF && StartState->g_val==INF)))
        {

            if (isTimeLeft(planning_start_time, planning_time_limit) == false)
            {
                return;
            }

            std::shared_ptr<OpenListEntry> top_entry = open_list.top();
            open_list.pop();

            double tc = get_transition_cost(robot_map, collision_thresh, top_entry->state->xyloc, x_size, y_size);
            
            //need to check if entry's key has changed since being added to open_list
            std::pair<double, double> old_key = top_entry->stored_key;
            std::pair<double, double> actual_key = getKey(top_entry->state, epsilon, robotposeX, robotposeY);

            if (actual_key != old_key)
            {
                top_entry=nullptr;
                continue;
            }
            if (is_in_set(top_entry->state->xyloc,closed_list)==true)
            {
                top_entry=nullptr;
                continue;
            }
            std::shared_ptr<RobotState>RS_expand = top_entry->state;
            
            if (RS_expand->g_val == RS_expand->rhs_val)
            {
                closed_list.emplace(RS_expand->xyloc.first, RS_expand->xyloc.second);
                top_entry=nullptr;
                continue;
            }
            n_states_expanded++;
            //std::cout << "Expanding State: (" << RS_expand->xyloc.first << " , " << RS_expand->xyloc.second << ") rhs= " << RS_expand->rhs_val << ", g= " << RS_expand->g_val << std::endl;
          
            if (RS_expand->g_val > RS_expand->rhs_val)
            {
                RS_expand->g_val = RS_expand->rhs_val;
                closed_list.emplace(RS_expand->xyloc.first, RS_expand->xyloc.second);
                std::vector<std::pair<int, int>>nbrs=get_neighbors(RS_expand->xyloc, x_size, y_size);
                for (std::pair<int,int>pred_loc : nbrs )
                {
                    if (isTimeLeft(planning_start_time, planning_time_limit) == false)
                    {
                        return;
                    }
                    if(get_transition_cost(robot_map, collision_thresh, pred_loc, x_size, y_size)<collision_thresh)
                    {
                        std::shared_ptr<RobotState> RS_pred = find_or_create_state(pred_loc, ALL_STATES);
                            
                        UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size,epsilon,robotposeX,robotposeY);
                    }
                }
                
            }
            else 
            {
                RS_expand->g_val = INF;//make overconsistent
                std::vector<std::pair<int, int>>nbrs = get_neighbors(RS_expand->xyloc, x_size, y_size);
                for (std::pair<int, int>pred_loc : nbrs)
                {
                    if (isTimeLeft(planning_start_time, planning_time_limit) == false)
                    {
                        return;
                    }
                    if(get_transition_cost(robot_map, collision_thresh, pred_loc, x_size, y_size)<collision_thresh)
                    {
                        std::shared_ptr<RobotState> RS_pred = find_or_create_state(pred_loc, ALL_STATES);
                        UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size, epsilon, robotposeX, robotposeY);
                    }
                }
                UpdateState(RS_expand, open_list, closed_list, INCONS_list,ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size, epsilon, robotposeX, robotposeY);

            }
            while(!open_list.empty() && open_list.top()==nullptr)
            {
                open_list.pop();
            }
            while(!open_list.empty() && open_list.top()->state==nullptr)
            {
                open_list.pop();
            }
        }
        std::cout << "---------------------Exited planning loop, epsilon ="<<epsilon<<"------------------- " << std::endl;
        std::cout <<"Number of states Expanded= "<<n_states_expanded << std::endl;
        tot_n_states_expanded += n_states_expanded;
        if (all_epsilons[i] != 1.0)
        {
            curr_epsilon_idx = i + 1;
        }
        
        std::shared_ptr<RobotState>current_state = StartState;
        New_Path.emplace_back(StartState->xyloc.first, StartState->xyloc.second);
        bool path_found = false;
        std::unordered_set<std::pair<int, int>, PairHash>visited_locs;
        visited_locs.emplace(StartState->xyloc.first, StartState->xyloc.second);
        while (path_found==false) 
        {
            double min_cost= INF;
            std::shared_ptr<RobotState>best_succ;
            std::vector<std::pair<int, int>>nbrs = get_neighbors(current_state->xyloc, x_size, y_size);
            for (std::pair<int, int>succ_loc : nbrs)
            {
                std::shared_ptr<RobotState>succ = find_state(succ_loc, ALL_STATES);
                if (succ == nullptr)
                {
                    continue;
                }
                if (is_in_set(succ->xyloc, visited_locs) == true)
                {
                    continue;
                }
                double tc = get_transition_cost(robot_map, collision_thresh, succ->xyloc, x_size, y_size);
                if (tc >= collision_thresh)
                {
                    continue;
                }
                double succ_cost = succ->g_val + tc;

                if (succ_cost < min_cost)
                {
                    best_succ = succ;
                    min_cost = succ_cost;
                }
            }
            if (best_succ==nullptr)
            {
                break;
            }

            visited_locs.emplace(best_succ->xyloc.first, best_succ->xyloc.second);
            New_Path.emplace_back(best_succ->xyloc.first, best_succ->xyloc.second);
            
            current_state = best_succ;
            if (current_state->xyloc == GoalState->xyloc)
            {
                path_found = true;
            }
        }
        
        if (path_found == true)
        {
            Best_Path = New_Path;
            robot_moves = 0;
        }
        
        
    }
    

}


void Update_States_w_Sensor_Info(
    std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey>& open_list,
    std::unordered_set<std::pair<int, int>, PairHash>& closed_list,
    std::unordered_set<std::pair<int, int>, PairHash>& INCONS_list,
    std::shared_ptr<RobotState>& GoalState,
    std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash>& ALL_STATES,
    const std::vector<std::pair<int, int>>& robot_map_changes,
    int*& robot_map,
    const int &collision_thresh,
    const int &x_size,
    const int &y_size,
    const int &robotposeX,
    const int &robotposeY,
    const double& epsilon 
    )
    
{
    for (std::pair<int, int>change_loc : robot_map_changes)
    {
        std::shared_ptr<RobotState>changed_state = find_state(change_loc, ALL_STATES);
        if (changed_state == nullptr)
        {
            continue;
        }
        else if (get_transition_cost(robot_map, collision_thresh, change_loc, x_size, y_size)>=collision_thresh)
        {
            ALL_STATES.erase(change_loc);
            INCONS_list.erase(change_loc);
            changed_state=nullptr;
        }
    }

    for (std::pair<int, int>change_loc : robot_map_changes)
    {
        std::vector<std::pair<int, int>>nbrs = get_neighbors(change_loc, x_size, y_size);
        for (std::pair<int, int>pred_loc : nbrs)
        {
            std::shared_ptr<RobotState> RS_pred = find_state(pred_loc, ALL_STATES);
            if (RS_pred == nullptr)
            {
                continue;
            }
            //std::cout << "State Being Updated: (" << RS_pred->xyloc.first << " , " << RS_pred->xyloc.second << ")" << std::endl;
            UpdateState(RS_pred, open_list, closed_list, INCONS_list, ALL_STATES, GoalState, collision_thresh, robot_map, x_size, y_size, epsilon, robotposeX, robotposeY);
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
    static int curr_epsilon_idx;
    static double planning_time_limit;
    if (curr_time == 0)
    {
        std::cout << "Enter Planning Time Constraint (ms): ";
        std::cin >> planning_time_limit;
    }
    
    auto planning_start_time = std::chrono::high_resolution_clock::now();

    static std::vector<std::pair<int, int>> Best_Path;
    Best_Path.reserve(x_size * y_size);

    static int robot_moves;
    static int* robot_map = new int[x_size * y_size];

    static std::pair<int,int> GoalLoc;
    GoalLoc = std::make_pair(targetposeX, targetposeY);
    static std::shared_ptr<RobotState>GoalState;
    std::pair<int, int> StartLoc= std::make_pair(robotposeX, robotposeY);
    std::shared_ptr<RobotState>StartState;

    std::unordered_set<std::pair<int, int>, PairHash> closed_list;
    closed_list.reserve(x_size * y_size);
    static std::unordered_map<std::pair<int, int>, std::shared_ptr<RobotState>, PairHash> ALL_STATES;//consider making this std::vector<std::shared_ptr<RobotState>> all_states(x_size * y_size, nullptr);
    ALL_STATES.reserve(x_size * y_size);
    static std::priority_queue<std::shared_ptr<OpenListEntry>, std::vector<std::shared_ptr<OpenListEntry>>, CompareKey> open_list;

    
    static std::unordered_set<std::pair<int, int>, PairHash>INCONS_list;
    INCONS_list.reserve(x_size * y_size);
    
    
    std::vector<std::pair<int, int>> robot_map_changes = updaterobotmap_and_getchanges(robot_map, sensing_data, SENSING_RANGE, robotposeX,
        robotposeY, x_size, y_size, curr_time,GoalLoc,ALL_STATES);

    

    if (curr_time == 0)
    {
       robot_moves = 0;
       curr_epsilon_idx = 0;
       GoalState = find_or_create_state(GoalLoc, ALL_STATES);
       GoalState->rhs_val = 0.0;
       GoalState->g_val = INF;
       std::shared_ptr<OpenListEntry>entry=std::make_shared<OpenListEntry>(GoalState, getKey(GoalState, all_epsilons[0], robotposeX, robotposeY));
       
       open_list.push(entry);

       StartState = find_or_create_state(StartLoc, ALL_STATES);
       StartState->rhs_val = INF;
       StartState->g_val = INF;
       
       ComputeorImprovePath(open_list, closed_list,INCONS_list, ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY,
           curr_time, StartState, all_epsilons,
                planning_time_limit, planning_start_time, Best_Path,GoalState,target_steps,target_traj,targetposeX,targetposeY,curr_epsilon_idx,robot_moves);
    }
    else
    {
        StartState = find_state(StartLoc, ALL_STATES);
        if (!robot_map_changes.empty())
        {
           curr_epsilon_idx = 0;
           
           Update_States_w_Sensor_Info(open_list, closed_list, INCONS_list,GoalState,ALL_STATES, robot_map_changes, robot_map,
               collision_thresh, x_size, y_size, robotposeX, robotposeY,all_epsilons[0]);
           
           StartState->rhs_val = INF;
           StartState->g_val = INF;
           ComputeorImprovePath(open_list, closed_list, INCONS_list, ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
               planning_time_limit, planning_start_time, Best_Path, GoalState, target_steps, target_traj, targetposeX, targetposeY, curr_epsilon_idx,robot_moves);
        }
        else if (all_epsilons[curr_epsilon_idx] != 1.0)
        {
            
            
            ComputeorImprovePath(open_list, closed_list, INCONS_list, ALL_STATES, robot_map, collision_thresh, x_size, y_size, robotposeX, robotposeY, curr_time, StartState, all_epsilons,
                planning_time_limit, planning_start_time, Best_Path, GoalState, target_steps, target_traj, targetposeX, targetposeY, curr_epsilon_idx,robot_moves);
        }
        
    }
    
    robot_moves++;
    action_ptr[0] = Best_Path[robot_moves].first;
    action_ptr[1] = Best_Path[robot_moves].second;
    


    return;
}
