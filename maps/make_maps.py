import sys
if __name__=="__main__":
    if len(sys.argv)!=2:
        print("invalid number of arguments\nUsage ./make_maps.py [mapFile]")
    state="info"
    map_file=open(sys.argv[1])
    new_map=None
    line=map_file.readline()
    print(f"got first line:{line}")
    while line:
        if line.strip("\n")=="N":
            map_size=map_file.readline()
            map_size=map_size.split(",")
            map_size[0]=int(map_size[0])
            map_size[1]=int(map_size[1])
            print(f"map size :{map_size}")
        #if line.split()[0]=="C":
        #if line.split()[0]=="R":
        #if line.split()[0]=="T":
        line=map_file.readline()
