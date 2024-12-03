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
    print(len(robot_trajectory))
    return robot_trajectory

SPEEDUP = 5000

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualizer.py <map filename>")
        sys.exit(1)
    
    x_size, y_size, collision_threshold, robotX, robotY, target_trajectory, costmap, robotmap = parse_mapfile(sys.argv[1])

    robot_trajectory = parse_robot_trajectory_file('../output/robot_trajectory.txt')

    fig, ax = plt.subplots()
    
    ax.imshow(robotmap)
    
    line1, = ax.plot([], [], lw=2, marker='o', color='b', label='robot')
    line2, = ax.plot([], [], lw=2, marker='o', color='r', label='target')
    
    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        return line1, line2
    
    # def update(frame):
    #     for i in range(x_size):
    #         for j in range(y_size):
    #             if(i>robotX-10 and i<robotX+10):
    #                 if(j>robotY-10 and j<robotY+10):
    #                     if(costmap[j][i] != robotmap[j][i]):
    #                         robotmap[j][i] = costmap[j][i]
    #     ax.imshow(robotmap)
        
    #     line1.set_data([p['x'] for p in robot_trajectory[frame] if 'x' in p], [p['y'] for p in robot_trajectory[frame] if 'y' in p])
        
    #     # t = robot_trajectory[frame+1]['t']
    #     # line2.set_data([p['x'] for p in target_trajectory[0]], [p['y'] for p in target_trajectory[0]])
    #     line2.set_data([p['x'] for p in target_trajectory[:frame+1]], [p['y'] for p in target_trajectory[:frame+1]])
    #     #plt.pause((robot_trajectory[frame+1]['t']-robot_trajectory[frame]['t'])/SPEEDUP)
        
    #     return line1, line2
    def update(frame):
        frame = frame*5
        robot_path = robot_trajectory[frame]  # This is the list of points for this trajectory
        # print("x_size"+ str(x_size))
        # print("y_size"+ str(y_size))
        # print("costmap")
        # print(costmap.shape)
        # print("robotmap")
        # print(robotmap.shape)
        # print(len(costmap[0]))
        if(len(robot_path)!=0):
            first_robot_position = robot_path[0]  # First dictionary in the list
            robotX = first_robot_position['x']
            robotY = first_robot_position['y']
            for i in range(x_size):
                # print("i: "+str(i))
                for j in range(y_size):
                    # print("j: "+str(j))
                    if (i > robotX - 200 and i < robotX + 200):
                        if (j > robotY - 200 and j < robotY + 200):
                            if (costmap[j][i] != robotmap[j][i]):
                                robotmap[j][i] = costmap[j][i]*10

        ax.imshow(robotmap)

        # Update robot position for this frame
        
        line1.set_data([p['x'] for p in robot_path], [p['y'] for p in robot_path])
        
        # Update target trajectory for this frame
        # line2.set_data([p['x'] for p in target_trajectory[:frame+1]], [p['y'] for p in target_trajectory[:frame+1]])
        line2.set_data([p['x'] for p in target_trajectory[:frame+1]], [p['y'] for p in target_trajectory[:frame+1]])

        return line1, line2
    
    ani = FuncAnimation(fig, update, frames=int(len(robot_trajectory)/5)-1, init_func=init, blit=False, interval=1)
    ani.save(filename = "Animation.mp4")
    print("UHHH done")
    plt.legend()
    plt.show()
    plt.savefig("map2_catch.png")