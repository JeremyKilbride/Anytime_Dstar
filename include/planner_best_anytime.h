#ifndef PLANNING_PROJECT_BEST_ANYTIME_H
#define PLANNING_PROJECT_BEST_ANYTIME_H

void planner(
    int* sensing_data, // Local map
    int SENSING_RANGE,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr);

#endif
