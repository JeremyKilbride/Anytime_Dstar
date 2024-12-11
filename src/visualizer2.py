import numpy as np 
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
import sys

def parse_mapfile(filename):
    with open(filename, 'r') as file:
        collision_threshold = 1
        assert file.readline().strip() == 'N', "Expected 'N' in the first line"
        x_size, y_size = map(int, file.readline().strip().split(','))
        
        assert file.readline().strip() == 'R', "Expected 'R' in the third line"
        robotX, robotY = map(int, file.readline().strip().split(','))
        
        assert file.readline().strip() == 'G', "Expected 'G' in the fifth line"
        target_trajectory = []
        line = file.readline().strip()
        while line != 'M':
            x, y = map(float, line.split(','))
            target_trajectory.append({'x': x, 'y': y})
            line = file.readline().strip()

        costmap = []
        robotmap=[]
        for line in file:
            row = list(map(float, line.strip().split(',')))
            costmap.append(row)
            row = [0 for _ in row]
            robotmap.append(row)
        
        costmap = np.asarray(costmap).T
        robotmap = np.asarray(robotmap).T
    
    return x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap, robotmap

# def parse_robot_trajectory_file(filename):
#     robot_trajectory = []
#     trajnum =-1
#     with open(filename, 'r') as file:
#         for line in file:
#             x, y = map(int, line.strip().split(','))
#             if(x!=-1):
#                 robot_trajectory[trajnum].append({'x': x, 'y': y})
#             else:
#                 robot_trajectory.append([])
#                 trajnum = trajnum +1
                
    
#     return robot_trajectory
def parse_robot_trajectory_file(filename):
    robot_trajectory = [[]]  # Start with an empty list for the first trajectory
    trajnum = 0
    with open(filename, 'r') as file:
        for line in file:
            x, y = map(int, line.strip().split(','))
            if x != -1:
                robot_trajectory[trajnum].append({'x': x, 'y': y})
            else:
                robot_trajectory.append([])  # Start a new trajectory
                trajnum += 1
    
    return robot_trajectory

SPEEDUP = 5000

# if __name__ == "__main__":
#     if len(sys.argv) != 4:
#         print("Usage: python visualizer.py <map filename> [sensorRange] [SpeedUp]")
#         sys.exit(1)
#     sensor_range=int(sys.argv[2])
#     SpeedUp = int(sys.argv[3])
#     x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap, robotmap = parse_mapfile(sys.argv[1])

#     robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')

#     fig, ax = plt.subplots()
    
#     # ax.imshow(robotmap)
#     ax.imshow(costmap)
#     line1, = ax.plot([], [], lw=2, marker='o', color='b', label='robot')
#     line2, = ax.plot([], [], lw=2, marker='o', color='r', label='target')
#     line3, = ax.plot([], [], lw=2, marker='o', color='g', label='robot')
    

#     line1.set_data([], [])
#     line2.set_data([], [])
#     line3.set_data([],[])
        
    

    # def update(frame):
    #     frame = frame*SpeedUp
    #     robot_path = robot_trajectory[frame]  # This is the list of points for this trajectory

    #     # if(len(robot_path)!=0):
    #     #     first_robot_position = robot_path[0]  # First dictionary in the list
    #     #     robotX = first_robot_position['x']
    #     #     robotY = first_robot_position['y']
    #     #     line3.set_data([robotX],[robotY])
    #     #     for i in range(x_size):
    #     #         # print("i: "+str(i))
    #     #         for j in range(y_size):
    #     #             # print("j: "+str(j))
    #     #             if (i > robotX - sensor_range and i < robotX + sensor_range):
    #     #                 if (j > robotY - sensor_range and j < robotY + sensor_range):
    #     #                     if (costmap[j][i] != robotmap[j][i]):
    #     #                         robotmap[j][i] = costmap[j][i]*10

        

    #     # Update robot position for this frame
        
    #     line1.set_data([p['x'] for p in robot_path], [p['y'] for p in robot_path])
        
    #     # Update target trajectory for this frame
    #     # line2.set_data([p['x'] for p in target_trajectory[:frame+1]], [p['y'] for p in target_trajectory[:frame+1]])
    #     line2.set_data([p['x'] for p in target_trajectory[:frame+1]], [p['y'] for p in target_trajectory[:frame+1]])
        

    #     return line1, line2, line3
        
    
    # for i in range(int(len(robot_trajectory)/SpeedUp)):
        
    #     update(i)


    # ani = FuncAnimation(fig, update, frames=int(len(robot_trajectory)/SpeedUp)-1, init_func=init, blit=False, interval=1)
    # ani.save(filename = "Animation2.mp4")




    # print("UHHH done")
    # plt.legend()
    # plt.show()

# import sys
# import matplotlib.pyplot as plt

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python visualizer.py <map filename> [sensorRange] [SpeedUp]")
        sys.exit(1)

    sensor_range = int(sys.argv[2])
    SpeedUp = int(sys.argv[3])
    x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap, robotmap = parse_mapfile(sys.argv[1])

    robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')

    fig, ax = plt.subplots()
    ax.imshow(costmap)

    # Initialize lines
    line1_x, line1_y = [], []
    line2_x, line2_y = [], []
    line3_x, line3_y = [], []

    # Plot initial empty lines
    line1, = ax.plot(line1_x, line1_y, lw=0, marker='o', markersize = 2, color='b', label='robot path')
    line2, = ax.plot(line2_x, line2_y, lw=0, marker='o', color='r',markersize = 2, label='target')
    line3, = ax.plot(line3_x, line3_y, lw=0, marker='o', color='g',markersize = 2, label='robot position')

    ax.legend()

    # Loop to update the plot incrementally
    for i in range(int(len(robot_trajectory) / SpeedUp)):
        # print(i)
        frame = i * SpeedUp
        if(frame==0):
            frame =1
        robot_path = robot_trajectory[frame]

        # Update robot path
        line1_x += [p['x'] for p in robot_path]
        line1_y += [p['y'] for p in robot_path]
        line1.set_data(line1_x, line1_y)

        # Update target trajectory
        line2_x += [p['x'] for p in target_trajectory[:frame+1]]
        line2_y += [p['y'] for p in target_trajectory[:frame+1]]
        line2.set_data(line2_x, line2_y)

        # Optionally add a static robot position update
        if robot_path:
            first_robot_position = robot_path[0]
            line3_x.append(first_robot_position['x'])
            line3_y.append(first_robot_position['y'])
            line3.set_data(line3_x, line3_y)

        # Refresh the plot to show updated data
        plt.draw()
        # plt.pause(0.1)  # Pause to visually see updates

    # Finalize and show the plot
    plt.show()

plt.savefig("../maps/map9_1_1.png")