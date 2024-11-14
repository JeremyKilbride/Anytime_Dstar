#include "planner.h"
#include<iostream>

using std::cout;

void plannerAstar()
{
    cout<<"using A* planner\n";
}

void plannerDstarLite()
{
    cout<<"using D* Lite planner\n";
    
}

void plannerAnytimeDstar()
{
    cout<<"using Anytime D* planner\n";
    
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
        cout<<"usage: ./planner [mapfile] [whichPlanner]";
    }
    int which=std::stoi(argv[2]);

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
    }

    return 0;
}