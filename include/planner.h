#ifndef PLANNING_PROJECT_ANYTIME_D_STAR_H
#define PLANNING_PROJECT_ANYTIME_D_STAR_H

#include <unordered_map>
#include <queue>
#include <limits>


struct Node
{
    int x;
    int y;
    double g=std::numeric_limits<double>::max();
    double h=std::numeric_limits<double>::max();;
    double v=std::numeric_limits<double>::max();;
};

#endif