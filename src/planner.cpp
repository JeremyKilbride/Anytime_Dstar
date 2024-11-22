#include "planner.h"
#include<iostream>
#include <chrono>
#include <string>
#include <cctype>

using std::cout;

bool operator> (Node& lhs, Node& rhs)
{
    double lhs_f = lhs.g + lhs.h;
    double rhs_f = rhs.g + rhs.h;
    return lhs_f > rhs_f;
}

void plannerAstar()
{
    std::chrono::steady_clock::time_point t_start =std::chrono::steady_clock::now();
    cout<<"using A* planner\n";
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
        plannerAstar();
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
