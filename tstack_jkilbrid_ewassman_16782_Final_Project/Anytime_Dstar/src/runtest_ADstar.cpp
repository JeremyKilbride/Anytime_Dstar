/*=================================================================
 *
 * runtest_ADstar.cpp
 *
 *=================================================================*/
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "../include/planner_ADstar.h"

#ifndef MAPS_DIR
#define MAPS_DIR "maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "output"
#endif
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))
int runtest_ADstar(int argc, char *argv[])
{
    int SENSING_RANGE;
    std::string str_sensor_range = argv[3];
    if (std::all_of(str_sensor_range.begin(), str_sensor_range.end(), ::isdigit)) {
        SENSING_RANGE = std::stoi(argv[3]);
        if (0 >= SENSING_RANGE) {
            std::cout << "sensor range must be greater than 0"<< std::endl;
            return 0;
        }
    }
    else {
        std::cout << "please use an integer to select the sensor range"<< std::endl;
        return 0;
    }

    /*std::cout << "Enter Sensing Range: ";
    std::cin >> SENSING_RANGE;*/
    // READ PROBLEM
    
    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;

    std::ifstream myfile;
    myfile.open(mapFilePath);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file:" << mapFilePath << std::endl;
        return -1;
    }

    // read map size
    char letter;
    std::string line;
    int x_size, y_size;

    myfile >> letter;
    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> x_size >> letter >> y_size;
    std:: cout << "map size: " << x_size << letter << y_size << std::endl;

    // read collision threshold
    int collision_thresh;
    myfile >> letter;
    if (letter != 'C')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> collision_thresh;
    std:: cout << "collision threshold: " << collision_thresh << std::endl;

    // read robot position
    int robotposeX, robotposeY;
    myfile >> letter;
    if (letter != 'R')
    {
        std::cout << "error parsing file" << std::endl;
        return -1;
    }
    
    myfile >> robotposeX >> letter >> robotposeY;
    std:: cout << "robot pose: " << robotposeX << letter << robotposeY << std::endl;

    // read trajectory
    std::vector<std::vector<int>> traj;

    do {
        std::getline(myfile, line);
    } while (line != "T");

    while (std::getline(myfile, line) && line != "M")
    {
        std::stringstream ss(line);
        int num1, num2;
        ss >> num1 >> letter >> num2;
        traj.push_back({num1, num2});
    }

    int target_steps = traj.size();
    int* target_traj = new int[2*target_steps];
    for (size_t i = 0; i < target_steps; ++i)
    {
        target_traj[i] = traj[i][0];
        target_traj[i + target_steps] = traj[i][1];
    }

    std::cout << "target_steps: " << target_steps << std::endl;

    // read map
    int* map = new int[x_size*y_size];
    for (size_t i=0; i<x_size; i++)
    {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t j=0; j<y_size; j++)
        {
            double value;
            ss >> value;

            map[j*x_size+i] = (int) value;
            if (j != y_size-1) ss.ignore();
        }
    }

    myfile.close();
    std::cout << "\nRunning planner" << std::endl;

    // CONTROL LOOP
    int curr_time = 0;
    int* action_ptr = new int[2];
    int targetposeX, targetposeY;
    int newrobotposeX, newrobotposeY;

    int numofmoves = 0;
    bool caught = false;
    int pathcost = 0;

    std::string outputDir = OUTPUT_DIR;
    std::string outputFilePath = outputDir + "/robot_trajectory.txt";
    std::ofstream output_file(outputFilePath);
    std::cout << "Writing robot trajectory to: " << outputFilePath << std::endl;
    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file: " << "../output/robot_trajectory.txt" << std::endl;
        return 1;
    }

    output_file << curr_time << "," << robotposeX << "," << robotposeY << std::endl;
   
    

    while (true)
    {
        auto start = std::chrono::high_resolution_clock::now();
        if (curr_time==0)
        {
            targetposeX = target_traj[curr_time];
            targetposeY = target_traj[curr_time + target_steps];
        }

        int min_x_sensor = std::max(1, robotposeX - SENSING_RANGE);
        int max_x_sensor = std::min(x_size, robotposeX + SENSING_RANGE);
        int min_y_sensor = std::max(1, robotposeY - SENSING_RANGE);
        int max_y_sensor = std::min(y_size, robotposeY + SENSING_RANGE);

        // Calculate the size of the partial map
        int sensor_data_x_size = max_x_sensor - min_x_sensor + 1;
        int sensor_data_y_size = max_y_sensor - min_y_sensor + 1;

        // Create the partial map array
        int* sensor_data = new int[sensor_data_x_size * sensor_data_y_size];

        // Extract the map values
        for (int i = 0; i < sensor_data_x_size; ++i) {
            for (int j = 0; j < sensor_data_y_size; ++j) {
                int global_x = min_x_sensor + i;
                int global_y = min_y_sensor + j;
                int local_x = i + 1;
                int local_y = j + 1;
                int map_idx_local = GETMAPINDEX(local_x, local_y, sensor_data_x_size, sensor_data_y_size);
                sensor_data[map_idx_local] = map[GETMAPINDEX(global_x, global_y, x_size, y_size)];
                
            }
        }

        
        planner_ADstar(
            sensor_data,
            SENSING_RANGE,
            2,
            x_size,
            y_size,
            robotposeX,
            robotposeY, 
            target_steps,
            target_traj,
            targetposeX, 
            targetposeY, 
            curr_time,
            action_ptr);
        newrobotposeX = action_ptr[0];
        newrobotposeY = action_ptr[1];

        if (newrobotposeX < 1 || newrobotposeX > x_size || newrobotposeY < 1 || newrobotposeY > y_size)
        {
            std::cout << "ERROR: out-of-map robot position commanded\n" << std::endl;
            return -1;
        }

        if (map[(newrobotposeY-1)*x_size + newrobotposeX-1] >= collision_thresh)
        {
            std::cout << "ERROR: planned action leads to collision\n" << std::endl;
            return -1;
        }

        if (abs(robotposeX-newrobotposeX)>1 || abs(robotposeY-newrobotposeY)>1)
        {
            std::cout << "ERROR: invalid action commanded. robot must move on 8-connected grid.\n" << std::endl;
            return -1;
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

        int movetime = std::max(1, (int)std::ceil(duration));

        if (newrobotposeX == robotposeX && newrobotposeY == robotposeY)
            numofmoves -= 1;
        
        // if (curr_time + movetime >= target_steps)
        //     break;
        
        curr_time = curr_time + movetime;
        numofmoves = numofmoves + 1;
        pathcost = pathcost + movetime*map[(robotposeY-1)*x_size + robotposeX-1];

        robotposeX = newrobotposeX;
        robotposeY = newrobotposeY;

        output_file << curr_time << "," << robotposeX << "," << robotposeY << std::endl;

        // check if target is caught
        float thresh = 0.5;
        // targetposeX = target_traj[curr_time];
        // targetposeY = target_traj[curr_time + target_steps];
        if (abs(robotposeX - targetposeX) <= thresh && abs(robotposeY-targetposeY) <= thresh)
        {
            caught = true;
            break;
        }
    }

    output_file.close();

    std::cout << "\nRESULT" << std::endl;
    std::cout << "target caught = " << caught << std::endl;
    std::cout << "time taken (s) = " << curr_time << std::endl;
    std::cout << "moves made = " << numofmoves << std::endl;
    std::cout << "path cost = " << pathcost << std::endl;

    delete target_traj;
    delete map;

    return 0;
}
