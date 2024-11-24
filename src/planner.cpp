#include "planner.h"
#include<iostream>
#include <chrono>
#include <string>
#include <cctype>

using std::cout;
void compute_h(Node& s){//TODO:update this
    s.h=0;
}

//function used by d* to calculate g val of a state
void update_node(Graph g, Node& s,int x_size, int y_size, int* map){
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    double best_g=std::numeric_limits<double>::max();
    //loop over all successors 
    for(int i=0;i<8;++i){
        int idx=get_key(x_size,y_size,s.x+dx[i],s.y+dy[i]);
        int map_val=map[idx];
        double cost;
        if (map_val==0){
            cost=1;
        }
        else{
            cost=std::numeric_limits<double>::max();
        }
        std::shared_ptr<Node> ptr_successor=g.get(idx);
        //check successor if null
        if (ptr_successor){
            double g=ptr_successor->v+cost;
            if(g<best_g){
                best_g=g;
            }
        }
        //push new states into graph        
        else{
            Node new_state(s.x+dx[i],s.y+dy[i]);
            compute_h(new_state);
            g.addNode(new_state);
        }
    }
    s.g=best_g;
}


bool operator> (const Node& lhs,const Node& rhs)
{
    double lhs_f = lhs.g + lhs.h;
    double rhs_f = rhs.g + rhs.h;
    return lhs_f > rhs_f;
}


void plannerAstar(int* map, int x_size, int y_size, Node start, Node goal)
{
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using A* planner\n";
    std::cout << "\n"<< "x_size: " << x_size << "\n";
    std::cout << "\n"<< "y_size: " << y_size << "\n";
    std::cout << "\n"<< "robot pose: " << start.x <<","<<start.y << "\n";
    std::cout << "\n"<< "goal pose: " << goal.x <<","<<goal.y << "\n";
    




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
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    Graph g(x_size,y_size);
    int num_expanded=0;
    //while f_goal > min f in open
    while(start>open_list.top() && !open_list.empty()){
        //get top item from open 
        Node state=open_list.top();
        open_list.pop();
        //state is consistent or overconsistent
        if (state.v>=state.g){
            state.v=state.g;
            //expand state
            for(int i=0;i<8;++i){
                int idx=get_key(x_size,y_size,state.x+dx[i],state.y+dy[i]);
                int map_val=map[idx];
                double cost;
                if (map_val==0){
                    cost=1;
                }      
                else{
                    cost=std::numeric_limits<double>::max();
                }
                //get successor
                std::shared_ptr<Node> ptr_successor=g.get(idx);
                if (ptr_successor){
                    if(ptr_successor->g > state.g + cost){
                        ptr_successor->g = state.g + cost;
                        //if g value lowered and successor not in closed insert into open 
                        if (closed_list.find(idx)!=closed_list.end()){
                            open_list.emplace(*ptr_successor);
                        }
                        //else if g lowered and in closed insert into incons
                        else{
                            inconsistent_list.emplace(idx,*ptr_successor);
                        }
                    }
                }   
                //successor not in graph, so create new node and emplace in graph
                else{
                    Node new_node(state.x+dx[i],state.y+dy[i]);
                    compute_h(new_node);
                    g.addNode(new_node);
                    open_list.emplace(new_node); 
                }
            }
        }
        //state is under consistent
        else{
            state.v=std::numeric_limits<double>::max();
            //update state and all successors
            update_node(g,state,x_size,y_size,map);
            open_list.emplace(state); 
            for (int i=0;i<8;++i){
                int idx=get_key(x_size,y_size,state.x+dx[i],state.y);
                std::shared_ptr<Node> state_ptr=g.get(idx);
                if (state_ptr){
                    update_node(g,*state_ptr,x_size,y_size,map);
                } 
                else{
                    Node new_node(state.x+dx[i],state.y);
                    compute_h(new_node);
                    g.addNode(new_node);
                }
            }
        }
    }

    //TODO:implement backtracking
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
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
    if (argc!=3){
        cout<<"invalid number of arguments\n";
        cout<<"usage: ./planner [mapfile] [whichPlanner]\n";
        return 0;
    }
    std::string map_path=argv[1];
    cout<<"got map file "<<map_path<<"\n";
    int which=-1;
    if (std::isdigit(argv[2][0])){
	which=std::stoi(argv[2]);
    }
    else{
	cout<<"please use an integer to select which planner\n";
	return 0;
    }

    //make variables necessary to setup problem
    int* map;
    int x_size;
    int y_size;
    Node start_node;
    Node goal_node;
    bool success=read_map(map_path,map,x_size,y_size,start_node,goal_node);
    if (!success){
        cout<< "did not read map successfully, exiting planner\n";
        return 0;
    }
    switch (which)
    {
    case planner::ASTAR:
        plannerAstar(map,x_size,y_size,start_node,goal_node);
        break;
    case planner::DSTAR_LITE:
        plannerDstarLite(map,x_size,y_size,start_node,goal_node);
        break;
    case planner::ANYTIME_DSTAR:
        plannerAnytimeDstar();
        break;
    default:
        cout<<"planner selection not recognized, please try again with either 0, 1, or 2\n";
    }

    return 0;
}
