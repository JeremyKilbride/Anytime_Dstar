#ifndef PLANNER_ADstar_H
#define PLANNER_ADstar_H

// Declare the plan function
void planner_ADstar(
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
    int* action_ptr
    );

#endif // PLANNER_ADstar_H