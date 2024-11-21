import sys
import numpy as np
import matplotlib.pyplot as plt
if __name__=="__main__":
    if len(sys.argv)!=3:
        print("invalid number of arguments\nUsage ./make_maps.py [mapFile] [newMapFileName]")
    state="info"
    map_file=open(sys.argv[1])
    new_map=None
    line=map_file.readline()
    binary_map=[]
    traj=[]
    while line:
        if line.strip("\n")=="N":
            map_size=map_file.readline()
            map_size=map_size.split(",")
            map_size[0]=int(map_size[0])
            map_size[1]=int(map_size[1])
            print(f"map size :{map_size}")
        #if line.split()[0]=="C":
        if line.strip("\n")=="R":
            robot_start=map_file.readline()
            robot_start=robot_start.split(",")
            robot_start[0]=int(robot_start[0])
            robot_start[1]=int(robot_start[1])
            print(f"start position: {robot_start}")
        if line.strip("\n")=="T":
            line=map_file.readline()
            state="Traj"
        if line.strip("\n")=="M":
            line=map_file.readline()
            state="map"
        if state=="Traj":
            traj.append(line.strip("\n"))
        if state=="map":
            map_line=line.strip("\n")
            map_line=map_line.split(",")
            binary_map_line=[]
            for val in map_line:
                val=float(val)
                if val > 1.0:
                    binary_map_line.append(1)
                else:
                    binary_map_line.append(0)
            binary_map.append(binary_map_line)
        line=map_file.readline()
    goal=traj[-1]
    map_arr=np.array(binary_map)
    np.savetxt(sys.argv[2],map_arr,fmt="%d",delimiter=",")
    new_map=open(sys.argv[2],"r")
    new_maplines=new_map.readlines()
    new_maplines.insert(0,"N\n")
    new_maplines.insert(1,str(map_size[0])+","+str(map_size[1])+"\n")
    new_maplines.insert(2,"R\n")
    new_maplines.insert(3,str(robot_start[0])+","+str(robot_start[1])+"\n")
    new_maplines.insert(4,"G\n")
    new_maplines.insert(5,goal+"\n")
    new_maplines.insert(6,"M\n")
    new_map=open(sys.argv[2],"w")
    new_map.writelines(new_maplines)
    print(f"got map with max: {np.max(map_arr)} and min {np.min(map_arr)}")
