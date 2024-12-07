import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__=="__main__":
    if len(sys.argv)!=2:
        print("invalid number of arguments\nUsage: python visualize_basic_map.py [mapFile]")
        exit()
    
    map_file=open(sys.argv[1])
    map_=[]
    line=map_file.readline()
    reading_map=False
    while line: 
        if line.strip("\n")=="N":
            line=map_file.readline()
            items=line.split(",")
            x_size=int(items[0])
            y_size=int(items[1])
            print(f"got grid size: {x_size} x {y_size}")
        if line.strip("\n")=="R":
            line=map_file.readline()
            items=line.split(",")
            robot_x=int(items[0])
            robot_y=int(items[1])
            print(f"got robot start: {robot_x}, {robot_y}")
        if line.strip("\n")=="G":
            line=map_file.readline()
            items=line.split(",")
            goal_x=int(items[0])
            goal_y=int(items[1])
            print(f"got goal: {goal_x}, {goal_y}")
        if line.strip("\n")=="M":
            reading_map=True
            line=map_file.readline()
        if reading_map:
            list_line=line.strip("\n").split(",")
            if (len(list_line) > 1):
                int_line=[int(i) for i in list_line]
                map_.append(int_line)
        line=map_file.readline()
    
    map_arr=np.array(map_)

    map_arr[robot_y,robot_x]=4
    map_arr[goal_y,goal_x]=3
    print(f"map:\n{map_arr}")
    plt.imshow(map_arr)
    # plt.scatter(robot_y,robot_x,50,"g")
    # plt.scatter(goal_y,goal_x,50,"r")
    plt.savefig("test.png")

