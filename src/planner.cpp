#include "planner.h"
#include<iostream>
#include <chrono>
#include <string>
#include <cctype>

using std::cout;

void plannerAstar(std::string map_name)
{
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
 




    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
}

void plannerDstarLite()
{
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using D* Lite planner\n";
    std::chrono::steady_clock::time_point t_end =std::chrono::steady_clock::now();
    std::chrono::microseconds planner_time = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    cout<<"total time: "<<(double)planner_time.count()/1000<< " ms\n";
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
