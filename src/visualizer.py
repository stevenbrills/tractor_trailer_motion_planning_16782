import matplotlib.pyplot as plt
import numpy as np

def parseFiileandCreateArray(filename):
    map_int = []
    with open(filename) as f:
        lines = f.readlines()
        h = lines[0].split(' ')[1]
        w = lines[1].split(' ')[1]
        map_str = lines[2:]
        map_int = []
        for line in map_str:
            map_int.append([int(x) for x in line.split(' ') if x != '\n'])
    #convert map_int to numpy array
    map_int = np.array(map_int)
    #display map_int as image
    plt.imshow(map_int)
    plt.show()


parseFiileandCreateArray('/home/naren/catkin_cl_rrt/tractor_trailer_motion_planning_16782/maps/map1.txt')