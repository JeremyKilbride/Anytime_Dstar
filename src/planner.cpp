#include "planner.h"
#include <iostream>
#include <chrono>
#include <string>
#include <cctype>
#include <queue>
#include <algorithm>

#define NUMOFDIRS 8

using std::cout;

std::vector<Node> get_successors(NodePtr s, Graph& g){
    std::vector<Node> successors;
    successors.reserve(8);
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    int s_idx=get_key(g._x_size(),s->x,s->y);
    for (int i=0;i<8;++i){ 
        int successor_x=s->x+dx[i];
        int successor_y=s->y+dy[i];
        int idx=get_key(g._x_size(),successor_x,successor_y);
        if(0<successor_x && successor_x<g._x_size() && 0<successor_y && successor_y<g._y_size()){
            NodePtr successor_ptr=g.getAddNode(idx);
            successor_ptr->parent_idx=s_idx;
            successors.emplace_back(*successor_ptr);
        }
    }
    return successors; 
}

//function used by d* to calculate g val of a state
void update_node(Graph& g, Node s,int x_size, int y_size, int* map){
    double best_g=std::numeric_limits<double>::max();
    std::vector<Node> successors=get_successors(std::make_shared<Node>(s),g);
    //loop over all successors 
    for(Node successor: successors){
        int idx=get_key(x_size,successor.x,successor.y);
        int map_val=map[idx];
        double cost;
        if (map_val==0){
            cost=1;
        }
        else{
            cost=std::numeric_limits<double>::max();
        }
        double g=successor.v+cost;
        if(g<best_g){
            best_g=g;
        }
    }
    s.g=best_g;
    g.set(s);
}


bool operator> (const Node& lhs,const Node& rhs)
{
    double lhs_f = lhs.g + lhs.h;
    double rhs_f = rhs.g + rhs.h;
    return lhs_f > rhs_f;
}

double computeHeuristic(Node current, Node goal_node){
    return sqrt((goal_node.x - current.x)*(goal_node.x-current.x)+(goal_node.y-current.y)*(goal_node.y-current.y));
};


void plannerAstar(int* map, int x_size, int y_size, Node start, Node goal)
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using A* planner\n";
    std::cout << "\n"<< "x_size: " << x_size << "\n";
    std::cout << "\n"<< "y_size: " << y_size << "\n";
    std::cout << "\n"<< "robot pose: " << start.x <<","<<start.y << "\n";
    std::cout << "\n"<< "goal pose: " << goal.x <<","<<goal.y << "\n";
    std::cout << "\n"<< "first map entry " << map[0] << "\n";
    int closed[x_size*y_size] = {};
    Graph Astar_graph(x_size,y_size);
    std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node>>> open;
    open.push(std::make_shared<Node>(start));
    Astar_graph.addNode(start);
    Astar_graph.addNode(goal);
    start.h = computeHeuristic(start,goal);
    start.g =0;
    goal.h = 0;

    while(!closed[get_key(x_size,goal)] && !open.empty()){ 
            
        std::shared_ptr<Node> current = open.top();
        int s = get_key(x_size,current);
        
        if(!open.empty()){
            open.pop();
        }
        
        
        if(!closed[s]){ 
            
            closed[s] = 1; 
            
                
            int  y = current->y;
            int x = current->x;
            
            for(int dir = 0; dir < NUMOFDIRS; dir++){ 
                

                int xprime = x + dX[dir];
                int yprime = y + dY[dir];
                int primeIndex = get_key(x_size,xprime,yprime);
                if (xprime >= 0 && xprime <= x_size && yprime >= 0 && yprime <= y_size  && map[primeIndex]<1 && !closed[primeIndex]){  //if the next direction is in the map, below the collision threshold and not closed
                    
                    if(Astar_graph.get(xprime,yprime)==nullptr){
                        Node newNode(xprime,yprime);
                        newNode.h = computeHeuristic(newNode,goal);
                        newNode.parent_idx = get_key(x_size,x,y);
                        Astar_graph.addNode(newNode);
                    }

                    
                    //if the g(s) is greater than g(sprime)
                    if(Astar_graph.get(xprime,yprime)->g > current->g+1){ //only update the gvalues for valid moves
                        
                        Astar_graph.get(xprime,yprime)->g = current->g+1;
                        open.push(Astar_graph.get(xprime,yprime));

                        
                                
                    }
                        

                }

            }
            
        }
        
    }
    std::vector<std::pair<int,int>> plan;
    int start_idx = get_key(x_size,start);
    int goal_idx = get_key(x_size,goal);
    int current_idx=start_idx;
    while(current_idx!=goal_idx){
        NodePtr _current_state_ptr=Astar_graph.get(current_idx);
        current_idx=_current_state_ptr->parent_idx;
        plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);
    }
    NodePtr _current_state_ptr=Astar_graph.get(current_idx);
    plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);

    
    cout<<"got plan with length "<<plan.size()<<"\n";


    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
}

std::vector<std::pair<int,int>> plannerDstarLite(int* map, int x_size, int y_size, Node start, Node goal)
{
    std::vector<std::pair<int,int>> plan;
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using D* Lite planner\n";
    std::cout << "\n"<< "x_size: " << x_size << "\n";
    std::cout << "\n"<< "y_size: " << y_size << "\n";
    std::cout << "\n"<< "robot pose: " << start.x <<","<<start.y << "\n";
    std::cout << "\n"<< "goal pose: " << goal.x <<","<<goal.y << "\n";
    std::unordered_map<int, Node> closed_list;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::unordered_map <int, Node> inconsistent_list;
    goal.g=0;
    int start_idx= get_key(x_size,start.x, start.y);
    int goal_idx= get_key(x_size,goal.x, goal.y);
    goal.compute_h(start.x,start.y);
    start.compute_h(start.x,start.y);
    open_list.emplace(goal);
    Graph g(x_size,y_size);
    g.addNode(goal);
    g.addNode(start);
    g.set_start(start_idx);
    g.set_goal(start.x,start.y);
    int num_expanded=0;
    bool expanded_start=false;
    //while f_start > min f in open
   while(*g.get(start_idx)>open_list.top() && !open_list.empty()){
        //get top item from open 
        Node state=open_list.top();
        open_list.pop();
        std::shared_ptr<Node> state_ptr=g.get(state);
        
        //state is consistent or overconsistent
        if (state_ptr->v >= state_ptr->g){
            state_ptr->v = state_ptr->g;
            //expand state if not already expanded
            ++num_expanded;
            int state_idx=get_key(x_size,state_ptr->x,state_ptr->y);             
            std::vector<Node> successors=get_successors(state_ptr,g);
            for(Node successor: successors){
                int idx=get_key(x_size,successor.x,successor.y);    
                int map_val=map[idx];
                double cost;
                if (map_val==0){
                    cost=1;
                }      
                else{
                    cost=std::numeric_limits<double>::max();
                }
                if(successor.g > state_ptr->g + cost){
                    successor.g = state_ptr->g + cost;
                    g.set(successor);
                    //if g value lowered and successor not in closed insert into open 
                    if (closed_list.find(idx)==closed_list.end()){
                        open_list.emplace(successor);
                    }
                    //else if g lowered and in closed insert into incons
                    else{
                        inconsistent_list.emplace(idx,successor);
                    }
                }
            }
            //add state to closed_list after expansion
            closed_list.emplace(state_idx,*state_ptr); 
        }
        //state is under consistent
        else{
            state_ptr->v=std::numeric_limits<double>::max();
            //update state and all successors
            update_node(g,*state_ptr,x_size,y_size,map);
            open_list.emplace(*state_ptr);
            std::vector<Node> successors=get_successors(state_ptr,g);
            for (Node successor: successors){
                update_node(g,successor,x_size,y_size,map);
                open_list.emplace(successor);
            }
        }
    }

    int current_idx=start_idx;
    while(current_idx!=goal_idx){
        NodePtr _current_state_ptr=g.get(current_idx);
        current_idx=_current_state_ptr->parent_idx;
        plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);
    }
    NodePtr _current_state_ptr=g.get(current_idx);
    plan.emplace_back(_current_state_ptr->x,_current_state_ptr->y);

    
    cout<<"got plan with length "<<plan.size()<<"\n";
    
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"\nfinished D* lite\n"<<"number of states expanded: "<<num_expanded<<"\n";
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
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

enum planner{
    ASTAR,
    DSTAR_LITE,
    ANYTIME_DSTAR
};

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
    Node current_node(start_node.x,start_node.y);

    std::unordered_set<int> changes=update_map(robot_map,global_map,x_size,y_size,current_node.x,current_node.y);

    switch (which)
    {
    case planner::ASTAR:
        plannerAstar(global_map,x_size,y_size,start_node,goal_node);
        break;
    case planner::DSTAR_LITE:
        while (current_node.x != goal_node.x || current_node.y != goal_node.y){    
            plan=plannerDstarLite(global_map,x_size,y_size,current_node,goal_node);
            current_node.x=plan[1].first;
            current_node.y=plan[1].second;
            std::unordered_set<int> changes=update_map(robot_map,global_map,x_size,y_size,current_node.x,current_node.y);
        }
        break;
    case planner::ANYTIME_DSTAR:
        plannerAnytimeDstar();
        break;
    default:
        cout<<"planner selection not recognized, please try again with either 0, 1, or 2\n";
    }

    return 0;
}
