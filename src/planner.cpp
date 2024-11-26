#include "planner.h"
#include<iostream>
#include <chrono>
#include <string>
#include <cctype>

using std::cout;

std::vector<NodePtr> get_successors(NodePtr s, Graph& g){
    std::vector<NodePtr> successors;
    successors.reserve(8);
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    for (int i=0;i<8;++i){ 
        int successor_x=s->x+dx[i];
        int successor_y=s->y+dy[i];
        int idx=get_key(g._x_size(),g._y_size(),successor_x,successor_y);
        if(0<successor_x && successor_x<g._x_size() && 0<successor_y && successor_y<g._y_size()){
            NodePtr successor_ptr=g.getAddNode(idx);
            successors.emplace_back(successor_ptr);
        }
    }
    return successors; 
}

//function used by d* to calculate g val of a state
void update_node(Graph& g, NodePtr s,int x_size, int y_size, int* map){
    double best_g=std::numeric_limits<double>::max();
    std::vector<NodePtr> successors=get_successors(s,g);
    //loop over all successors 
    for(NodePtr ptr_successor: successors){
        int idx=get_key(x_size,y_size,ptr_successor->x,ptr_successor->y);
        int map_val=map[idx];
        double cost;
        if (map_val==0){
            cost=1;
        }
        else{
            cost=std::numeric_limits<double>::max();
        }
        double g=ptr_successor->v+cost;
        if(g<best_g){
            best_g=g;
        }
    }
    s->g=best_g;
    g.set(*s);
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
    int start_idx= get_key(x_size,y_size,start.x, start.y);
    goal.compute_h();
    start.compute_h();
    open_list.emplace(goal);
    int dx[8]={0,1,0,-1,1,-1,-1,1};
    int dy[8]={1,0,-1,0,1,-1,1,-1};
    Graph g(x_size,y_size);
    g.addNode(goal);
    g.addNode(start);
    int num_expanded=0;
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
            int state_idx=get_key(x_size,y_size,state_ptr->x,state_ptr->y);    
            //if (closed_list.find(state_idx)==closed_list.end()){
                std::vector<NodePtr> successors=get_successors(state_ptr,g);
                for(NodePtr ptr_successor: successors){
                    int idx=get_key(x_size,y_size,ptr_successor->x,ptr_successor->y);    
                    int map_val=map[idx];
                    double cost;
                    if (map_val==0){
                        cost=1;
                    }      
                    else{
                        cost=std::numeric_limits<double>::max();
                    }
                    if(ptr_successor->g > state_ptr->g + cost){
                        ptr_successor->g = state_ptr->g + cost;
                        ptr_successor->parent_idx=state_idx;
                        g.set(*ptr_successor);
                        //if g value lowered and successor not in closed insert into open 
                        if (closed_list.find(idx)==closed_list.end()){
                            open_list.emplace(*ptr_successor);
                        }
                        //else if g lowered and in closed insert into incons
                        else{
                            inconsistent_list.emplace(idx,*ptr_successor);
                        }
                    }
                }
                //add state to closed_list after expansion
                closed_list.emplace(state_idx,*state_ptr);   
            }
        //}
        //state is under consistent
        else{
            state_ptr->v=std::numeric_limits<double>::max();
            //update state and all successors
            update_node(g,state_ptr,x_size,y_size,map);
            open_list.emplace(*state_ptr);
            std::vector<NodePtr> successors=get_successors(state_ptr,g);
            for (NodePtr successor_ptr: successors){
                update_node(g,successor_ptr,x_size,y_size,map);
                open_list.emplace(*successor_ptr);
            }
        }
    }

    //TODO:implement backtracking

    Node found_start=closed_list[start_idx];
    
    cout<<"got start state "<<found_start.x<<", "<<found_start.y<<"\n";
    
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
