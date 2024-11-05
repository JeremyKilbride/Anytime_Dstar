#ifndef PLANNING_PROJECT_ANYTIME_D_STAR_H
#define PLANNING_PROJECT_ANYTIME_D_STAR_H

#include <unordered_map>
#include <queue>
#include <math.h>

struct Node
{
    int x;
    int y;
    double g=INT32_MAX;
    double h=INT32_MAX;
    double v=INT32_MAX;
};

#endif