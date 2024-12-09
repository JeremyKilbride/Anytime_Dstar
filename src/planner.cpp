#include "planner.h"
#include "planner_best_anytime.h"
#include <iostream>
#include <chrono>
#include <string>
#include <cctype>
#include <queue>
#include <algorithm>
#include <filesystem>
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "output"
#endif

#define NUMOFDIRS 8

using std::cout;

std::vector<Node> get_successors(NodePtr s, Graph& g, int* map){
    std::vector<Node> successors;
    successors.reserve(8);
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    int s_idx=get_key(g._x_size(),s->x,s->y);
    for (int i=0;i<8;++i){ 
        int successor_x=s->x+dx[i];
        int successor_y=s->y+dy[i];
        int idx=get_key(g._x_size(),successor_x,successor_y);
        if(0<=successor_x && successor_x<g._x_size() && 0<=successor_y && successor_y<g._y_size() && map[idx]==0){
            NodePtr successor_ptr=g.getAddNode(idx);
            successor_ptr->parent_idx=s_idx;
            successors.emplace_back(*successor_ptr);
        }
    }
    return successors; 
}
//a find function that only checks if a node's x and y vals are equivalent, rather than all member variables
std::set<Node>::iterator find_node(const std::set<Node,std::less<Node>>& open,const Node& s){
    
    for(std::set<Node>::iterator it=open.begin();it!=open.end();++it){
        if(it->x==s.x &&it->y==s.y){
            return it;
        }
    }
    return open.end();
}

//function used by d* update rhs value of a node and determine its membership in open list
void update_node(Graph& g, 
                 Node s,
                 int x_size, 
                 int y_size, 
                 int* map, 
                 std::set<Node,std::less<Node>>& open){

    if(!s.is_goal){
        double best_rhs=std::numeric_limits<double>::max();
        std::vector<Node> successors=get_successors(std::make_shared<Node>(s),g, map);
        int idx=get_key(x_size,s.x,s.y);
        int map_val=map[idx];
        //loop over all successors 
        for(Node successor: successors){
            int successor_idx=get_key(x_size,successor.x,successor.y);
            int successor_map_val=map[successor_idx];
            double cost;
            if (successor_map_val==0 && map_val==0){
                cost=1;
            }
            else{
                cost=std::numeric_limits<double>::max();
            }
            double rhs=successor.g+cost;
            if(rhs<best_rhs){
                best_rhs=rhs;
            }
        }
        s.rhs=best_rhs;
    }   
    auto it=find_node(open,s);
    if(it!=open.end()){
        open.erase(it);
    }
    if(s.g!=s.rhs){
        g.calculate_node_key(s);
        std::pair<std::set<Node>::iterator,bool>ret=open.emplace(s);
        if (!ret.second){
            cout<<"insertion failed\n";
        }
    }
    g.set(s);
}


bool operator< (const Node& lhs,const Node& rhs)
{
   if (lhs.key.first==rhs.key.first){
        if(lhs.key.second==rhs.key.second){
            if(lhs.x==rhs.x){
                return lhs.y < rhs.y;
            }
            else{
                return lhs.x < rhs.x;
            }
        }else{
            return lhs.key.second < rhs.key.second;
        }
    }else{
        return lhs.key.first < rhs.key.first;
    }
}

bool operator< (Node& lhs,Node& rhs)
{
   if (lhs.key.first==rhs.key.first){
        if(lhs.key.second==rhs.key.second){
            if(lhs.x==rhs.x){
                return lhs.y < rhs.y;
            }
            else{
                return lhs.x < rhs.x;
            }
        }else{
            return lhs.key.second < rhs.key.second;
        }
    }else{
        return lhs.key.first < rhs.key.first;
    }
}


double computeHeuristic(Node current, Node goal_node){
    return std::max(abs(goal_node.x - current.x),abs((goal_node.y-current.y)));
    // return 0;
};

//compare node structure for the priority queue
struct CompareNode {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        // Return true if 'a' should come after 'b', i.e., a min-heap
        return a->g + a->h > b->g+ b->h;  // For min-heap: smaller elements have higher priority
    }
};



std::vector<std::pair<int,int>> plannerAstar(int* map, int x_size, int y_size, Node start, Node goal,double& total_time, int& total_expanded)
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using A* planner\n";
    int states_expanded = 0;
    int* closed = new int[x_size*y_size]();
    Graph Astar_graph(x_size,y_size);
    std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node>>,CompareNode> open;
    Astar_graph.addNode(start);
    Astar_graph.addNode(goal);
    start.h = computeHeuristic(start,goal);
    start.g =0;
    goal.h = 0;

    open.push(std::make_shared<Node>(start));

    while(!open.empty()){ 
        // std::cout<<"\n"<<"open size: "<<(open.size())<<"\n";

        std::shared_ptr<Node> current = open.top();
        if(get_key(x_size,current)==get_key(x_size,goal)){
            
            break;
        }
        int s = get_key(x_size,current);
        
        if(!open.empty()){
            open.pop();
        }
        
        states_expanded ++;
        if(!closed[s]){ 
            
            closed[s] = 1; 
            
                
            int  y = current->y;
            int x = current->x;
            
            for(int dir = 0; dir < NUMOFDIRS; dir++){ 
                

                int xprime = x + dX[dir];
                int yprime = y + dY[dir];
                int primeIndex = get_key(x_size,xprime,yprime);
                if (xprime >= 0 && xprime < x_size && yprime >= 0 && yprime < y_size  && map[primeIndex]<1 && !closed[primeIndex]){  //if the next direction is in the map, below the collision threshold and not closed
                    
                    if(Astar_graph.get(xprime,yprime)==nullptr){
                        
                        Node newNode(xprime,yprime);
                        newNode.h = computeHeuristic(newNode,goal);
                        newNode.parent_idx = get_key(x_size,x,y);
                        Astar_graph.addNode(newNode);
                        
                    }

                    // std::cout<<"current g: "<< current->g<<"\n";
                    //if the g(s) is greater than g(sprime)
                    if(Astar_graph.get(xprime,yprime)->g > current->g+1){ //only update the gvalues for valid moves
                        Node newNode(xprime,yprime);
                        newNode.h = computeHeuristic(newNode,goal);
                        newNode.g = current->g+1;
                        // std::cout<<"newNode g: "<< newNode.g<<"\n";
                        newNode.parent_idx = get_key(x_size,x,y);
                        Astar_graph.set(newNode);
                        // std::cout<<"newNode g in map: "<< Astar_graph.get(xprime,yprime)->g<<"\n";
                        open.push(Astar_graph.get(xprime,yprime));

                        
                                
                    }
                        

                }

            }
            
        }
        
    }
    std::vector<std::pair<int,int>> plan;
    int start_idx = get_key(x_size,start);
    int goal_idx = get_key(x_size,goal);
    int current_idx=goal_idx;
    while(current_idx!=start_idx){
        NodePtr _current_state_ptr=Astar_graph.get(current_idx);
        current_idx=_current_state_ptr->parent_idx;
        plan.insert(plan.begin(),std::make_pair(_current_state_ptr->x,_current_state_ptr->y));
    }
    NodePtr _current_state_ptr=Astar_graph.get(current_idx);
    plan.insert(plan.begin(),std::make_pair(_current_state_ptr->x,_current_state_ptr->y));

    
    cout<<"got plan with length "<<plan.size()<<"\n";
    cout<<"states expanded: "<<states_expanded<<"\n";
    delete[] closed;
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
    total_time+=(double)planner_time.count()/1000;
    total_expanded+=states_expanded;
    return plan;
}

std::unordered_set<int>get_successor_idxs(Node& s, int x_size, int y_size, int* map){
    std::unordered_set<int> successors;
    successors.reserve(8);
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    for(int i=0; i<8; ++i){
        int cx=s.x+dx[i];
        int cy=s.y+dy[i];
        int s_idx=get_key(x_size,cx,cy);
        if(cx>=0 && cx<x_size && cy>=0 && cy<y_size && map[s_idx]==0){
            int idx=get_key(x_size, cx, cy);
            successors.insert(idx);
        }
    }
    return successors;
}

int get_best_neighbor_idx(Graph& g, Node& s,int* map,int x_size,int y_size){
    std::unordered_set<int> neighbor_idxs=get_successor_idxs(s,x_size,y_size, map);
    double best_cost=std::numeric_limits<double>::max();
    int best_idx=-1;
    for (int idx : neighbor_idxs){
        NodePtr ptr_node=g.getAddNode(idx);
        double cost;
        int val=map[idx];
        if(val==0){
            cost=1;
        }else{
            cost=std::numeric_limits<double>::max();
        }
        if((ptr_node->g+cost) <best_cost){
            best_cost=ptr_node->g+cost;
            best_idx=idx;
        }
    }
    return best_idx;
}

std::vector<std::pair<int,int>> plannerDstarLite(int* map, int x_size, int y_size, Node start, Node goal,Graph& g, std::set<Node, std::less<Node>>& open_list, double& total_time, int& total_expanded)
{
    std::vector<std::pair<int,int>> plan;
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    std::cout << "\n"<< "robot pose: " << start.x <<","<<start.y << "\n";
    std::cout << "\n"<< "goal pose: " << goal.x <<","<<goal.y << "\n";
    static bool first_time=true; 
    int start_idx= get_key(x_size,start.x, start.y);
    int goal_idx= get_key(x_size,goal.x, goal.y);
    start.compute_h(start.x,start.y);
    g.set_goal(start.x,start.y);
    g.addNode(start);
    if (first_time){
        goal.rhs=0;
        g.calculate_node_key(goal);
        goal.is_goal=true;
        open_list.emplace(goal);
        g.addNode(goal);
        first_time=false;
    }
    g.set_start(start_idx);
    int num_expanded=0;
    bool expanded_start=false;
    cout<<"open list size: "<<open_list.size()<<"\n";
    cout<<"start state map val "<<map[start_idx]<<"\n";
   while(*open_list.begin()<g.calculate_start_key() || g.get(start_idx)->rhs != g.get(start_idx)->g){
        //get top item from open 
        Node state=*open_list.begin();
        open_list.erase(open_list.begin());
        std::shared_ptr<Node> state_ptr=g.get(state);
        Node copy_new_km=state;
        g.calculate_node_key(copy_new_km);
        if(state<copy_new_km){
            g.set(copy_new_km);
            open_list.emplace(copy_new_km);
        }
        //state is overconsistent
        else if (state_ptr->g > state_ptr->rhs){
            ++num_expanded;
            state_ptr->g = state_ptr->rhs;
            g.set(*state_ptr);
            std::vector<Node> successors=get_successors(state_ptr,g,map);
            for(Node successor: successors){
                update_node(g, successor, x_size, y_size, map, open_list);
            }
        }
        //state is under consistent
        else{
            ++num_expanded;
            state_ptr->g=std::numeric_limits<double>::max();
            g.set(*state_ptr);
            //update state and all successors
            update_node(g,*state_ptr,x_size,y_size,map,open_list);
            std::vector<Node> successors=get_successors(state_ptr,g,map);
            for (Node successor: successors){
                update_node(g,successor,x_size,y_size,map,open_list);
            }
        }
    }

    cout<<"\nfinished D* lite\n"<<"number of states expanded: "<<num_expanded<<"\nbeginning back track\n";
    cout<<"goal node, g "<<g.get(goal_idx)->g<<" ,rhs "<<g.get(goal_idx)->rhs<<"\n";
    int current_idx=start_idx;
    while(current_idx!=goal_idx){
        NodePtr _current_state_ptr=g.get(current_idx);
        current_idx=get_best_neighbor_idx(g,*_current_state_ptr,map,x_size,y_size);
        plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);
    }
    NodePtr _current_state_ptr=g.get(current_idx);
    plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);
    cout<<"got plan with length "<<plan.size()<<"\n";
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
    total_time+=(double)planner_time.count()/1000;
    total_expanded+=num_expanded;
    return plan;
}

void plannerAnytimeDstar()
{
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using Anytime D* planner\n";
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
}



//checks if a state is within the robot's sensing range based in the robot's current position
bool check_in_sense_range(int cx,int cy, int x, int y, int sr,int offset){
    return (x>cx-sr-offset && x<cx+sr+offset && y>cy-sr-offset && y<cy+sr+offset);
}

//used to merge sets of indices for propagating map changes, only includes states within the sensor range of the robot
void merge_sets(std::unordered_set<int>& big, std::unordered_set<int>& small,int cx, int cy,int sensing_rng, int x_size, int offset ){
    for(int i: small){
        int x=i%x_size; 
        int y=i/x_size;
        if(big.find(i)==big.end() && check_in_sense_range(x,y,cx,cy,sensing_rng,offset)){
            big.insert(i);
        }
    }
}

enum planner{
    ASTAR,
    DSTAR_LITE,
    ANYTIME_DSTAR
};



void output(const std::vector<std::pair<int, int>>& plan) {
    std::string outputDir = OUTPUT_DIR;
    std::string outputFilePath = outputDir + "/robot_trajectory.txt";

    // Open file in append mode
    std::ofstream output_file(outputFilePath, std::ios::app);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file: " << outputFilePath << std::endl;
        return;
    }

    // Append trajectory points to the file
    for (const auto& point : plan) {
        output_file << point.first << "," << point.second << "\n";
    }

    // Append a marker to indicate the end of this trajectory
    output_file << -1 << "," << -1 << "\n";

    output_file.close();
    // std::cout << "Trajectory appended to " << outputFilePath << std::endl;
}

int main(int argc, char** argv)
{
    if (argc!=4){
        cout<<"invalid number of arguments\n";
        cout<<"usage: ./planner [mapfile] [whichPlanner] [sensorRange]\n";
        return 0;
    }
    std::string map_path=argv[1];    
    cout<<"got map file "<<map_path<<"\n";
    int which=-1;
    int sensing_range=0;
    if (std::isdigit(argv[2][0])){
	    which=std::stoi(argv[2]);
    }
    else{
	    cout<<"please use an integer to select which planner\n";
	return 0;
    }
    std::string str_sensor_range=argv[3];
    if (std::all_of(str_sensor_range.begin(), str_sensor_range.end(), ::isdigit )){
        sensing_range=std::stoi(argv[3]);
        if(0>sensing_range){
            cout<<"sensor range must be greater than 0\n";
            return 0;
        }
    }
    else{
        cout<<"please use an integer to select the sensor range\n";
        return 0;
    }
    


    std::string outputDir = OUTPUT_DIR;
    std::string outputFilePath = outputDir + "/robot_trajectory.txt";
    std::ofstream output_file(outputFilePath);
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file: " << outputFilePath << std::endl;
    
    }else{
        output_file << -1 << "," << -1 << "\n";
        output_file.close();

    }




    //make variables necessary to setup problem
    int* global_map;
    int* robot_map;
    int x_size;
    int y_size;
    Node start_node;
    Node goal_node;
    std::vector<std::pair<int,int>> plan;

    bool success=read_map(map_path,global_map,x_size,y_size,start_node,goal_node);
    if (!success){
        cout<< "did not read map successfully, exiting planner\n";
        return 0;
    }

    //make blank map for the robot
    robot_map = new int[x_size*y_size];
    for (int k=0; k<x_size; ++k){
        for(int l=0;l<y_size; ++l){
            int idx=get_key(x_size,k,l);
            robot_map[idx]=0;
        }
    }
    //make more variables necessary for solving the problem or tracking statistics
    Node current_node(start_node.x,start_node.y);
    std::unordered_set<int> changes=update_map(robot_map,global_map,x_size,y_size,current_node.x,current_node.y,sensing_range);
    Graph graph(x_size,y_size);
    std::unordered_map <int, Node> incons;
    int current_plan_idx=1;
    std::set<Node, std::less<Node>> _open;
    Node last_replan_node=current_node;
    int num_steps=0;
    double num_plans=0;
    double total_time=0;
    int total_expanded=0;

    switch (which)
    {
    case planner::ASTAR:
        // cout<<"sensing rang"<<sensing_range;
        plan=plannerAstar(robot_map,x_size,y_size,current_node,goal_node,total_time,total_expanded);
        ++num_plans;
        output(plan);
        while (current_node.x != goal_node.x || current_node.y != goal_node.y){
            current_node.x=plan[1].first;
            current_node.y=plan[1].second;
            changes=update_map(robot_map,global_map,x_size,y_size,current_node.x,current_node.y,sensing_range);
            ++num_steps;
            plan=plannerAstar(robot_map,x_size,y_size,current_node,goal_node,total_time,total_expanded);
            ++num_plans;
            output(plan);

            // std::cout<<"x: "<<current_node.x<<" y: "<<current_node.y<<"\n";
        }
        cout<<"\ngoal reached!!\n";
        cout<<"number of steps: "<<num_steps<<"\n";
        cout<<"average planning time: "<< total_time/(double)num_plans << " ms\n";
        cout<<"number of plans generated: "<<num_plans<<"\n";
        cout<<"average number of states expanded: "<<(double)total_expanded/num_plans<<"\n";

        break;
    case planner::DSTAR_LITE:
        cout<<"using D* Lite planner\n";
        std::cout << "\n"<< "x_size: " << x_size << "\n";
        std::cout << "\n"<< "y_size: " << y_size << "\n";
        plan=plannerDstarLite(robot_map,x_size,y_size,current_node,goal_node,graph,_open,total_time,total_expanded);
        ++num_plans;
        while (current_node.x != goal_node.x || current_node.y != goal_node.y){ 
            if (abs(current_node.x-plan[current_plan_idx].first)<=1 && abs(current_node.y-plan[current_plan_idx].second)<=1 ){
                cout<<"moving with plan idx "<<current_plan_idx<<"\n";
                current_node.x=plan[current_plan_idx].first;
                current_node.y=plan[current_plan_idx].second;
                ++current_plan_idx;
                ++num_steps;
            }else{
                cout<<"invalid move, exiting\n";
                cout<<"current plan idx "<<current_plan_idx<<"\n";
                cout<<"current position "<<current_node.x<<", "<<current_node.y<<", plan position "<<plan[current_plan_idx].first<<", "<<plan[current_plan_idx].second<<"\n";
                cout<<"previous plan item "<< plan[current_plan_idx-1].first<<", "<<plan[current_plan_idx-1].second<<"\n";
                return 0;
            }     
            int current_idx=get_key(x_size,current_node.x,current_node.y);
            std::unordered_set<int> changes=update_map(robot_map,global_map,x_size,y_size,current_node.x,current_node.y,sensing_range);
            if (!changes.empty()){
                //update changed nodes
                for(int change_idx: changes){
                    NodePtr ptr_changed=graph.getAddNode(change_idx);
                    update_node(graph,*ptr_changed,x_size,y_size,robot_map,_open);
                }
                graph.km+=computeHeuristic(last_replan_node,current_node);
                last_replan_node=current_node;
                //replan
                plan=plannerDstarLite(robot_map,x_size,y_size,current_node,goal_node,graph,_open,total_time,total_expanded);
                output(plan);
                ++num_plans;
                current_plan_idx=1;
            }      
        }
        cout<<"\ngoal reached!!\n";
        cout<<"number of steps: "<<num_steps<<"\n";
        cout<<"average planning time: "<< total_time/(double)num_plans << " ms\n";
        cout<<"number of plans generated: "<<num_plans<<"\n";
        cout<<"average number of states expanded: "<<(double)total_expanded/num_plans<<"\n";
        break;
    case planner::ANYTIME_DSTAR:
        plannerAnytimeDstar();
        break;
    default:
        cout<<"planner selection not recognized, please try again with either 0, 1, or 2\n";
    }

    return 0;
}
