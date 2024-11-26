#include "planner.h"
#include<iostream>
#include <chrono>
#include <string>
#include <cctype>
#include <queue>
#include <cmath>
#define NUMOFDIRS 8
using std::cout;


bool operator> (Node& lhs, Node& rhs)
{
    double lhs_f = lhs.g + lhs.h;
    double rhs_f = rhs.g + rhs.h;
    return lhs_f > rhs_f;
}

double computeHeuristic(Node current, Node goal_node){
    return sqrt((goal_node.x - current.x)*(goal_node.x-current.x)+(goal_node.y-current.y)*(goal_node.y-current.y));
};

void plannerAstar(std::string map_name)
{
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using A* planner\n";
    int* map;
    int x_size;
    int y_size;
    Node start_node;
    Node goal_node;
    read_map(map_name,map,x_size,y_size,start_node,goal_node);
    std::cout << "\n"<< "x_size: " << x_size << "\n";
    std::cout << "\n"<< "y_size: " << y_size << "\n";
    std::cout << "\n"<< "robot pose: " << start_node.x <<","<<start_node.y << "\n";
    std::cout << "\n"<< "goal pose: " << goal_node.x <<","<<goal_node.y << "\n";
    std::cout << "\n"<< "first map entry " << map[0] << "\n";
    int closed[x_size*y_size] = {};
    Graph Astar_graph(x_size,y_size);
    std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node>>> open;
    open.push(std::make_shared<Node>(start_node));
    Astar_graph.addNode(start_node);
    Astar_graph.addNode(goal_node);
    start_node.h = computeHeuristic(start_node,goal_node);
    start_node.g =0;
    goal_node.h = 0;

    while(!closed[get_key(x_size,goal_node)] && !open.empty()){ 
            
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
                        newNode.h = computeHeuristic(newNode,goal_node);
                    }

                    
                    //if the g(s) is greater than g(sprime)
                    if(Astar_graph.get(xprime,yprime)->g > current->g+1){ //only update the gvalues for valid moves
                        
                        Astar_graph.get(xprime,yprime)->g = current->g+1;
                        //Come in latrer and finish this up

                        
                                
                    }
                        

                }

            }
            
        }
        
    }




    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
}

std::vector<std::pair<int,int>> plannerDstarLite()
{
    std::vector<std::pair<int,int>> plan;
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using D* Lite planner\n";
    std::unordered_map<int, Node> closed_list;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;

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
    switch (which)
    {
    case planner::ASTAR:
        plannerAstar(map_path);
        break;
    case planner::DSTAR_LITE:
        plannerDstarLite();
        break;
    case planner::ANYTIME_DSTAR:
        plannerAnytimeDstar();
        break;
    default:
        cout<<"planner selection not recognized, please try again with either 0, 1, or 2\n";
    }

    return 0;
}
