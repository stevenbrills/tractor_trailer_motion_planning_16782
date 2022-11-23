import matplotlib.pyplot as plt
import numpy as np
from icecream import ic
#define TRACTOR_WIDTH (2.0)
#define L1 (5.0)
#define L2 (5.0)
#define BODY (1.0)
#define DIAMETER (0.5)
#define MAP_RESOLUTION (0.1)
#define RECT_RESOLUTION (0.1)
TRACTOR_WIDTH = 2.0
L1 = 1.0
L2 = 1.0
BODY = 1.0
DIAMETER = 0.5
MAP_RESOLUTION = 0.1
RECT_RESOLUTION = 0.1
def convertToPixels(x,y,theta,beta, h):
    tractor_coords = []
    trailer_coords = []
    theta = float(theta)
    beta = float(beta)
    steps=int(max(L1,L2)/MAP_RESOLUTION)
    for i in range(0, steps+1):
        x1 = int((x+i/steps*L2*np.cos(theta))/MAP_RESOLUTION)
        y1 = int((y+i/steps*L2*np.sin(theta))/MAP_RESOLUTION)
        trailer_coords.append([x1,y1])
        x2 = int((x+L2*np.cos(theta)+i/steps*L1*np.cos(theta+beta))/MAP_RESOLUTION)
        y2 = int((y+L2*np.sin(theta)+i/steps*L1*np.sin(theta+beta))/MAP_RESOLUTION)
        tractor_coords.append([x2,y2])

    ic(trailer_coords)
    ic(tractor_coords)  
    return tractor_coords, trailer_coords

def parseFiileandCreateArray(filename):
    map_int = []
    with open(filename) as f:
        lines = f.readlines()
        h = lines[0].split(' ')[1]
        w = lines[1].split(' ')[1]
        h = int(h)
        w = int(w)
        x,y,theta,beta = lines[2].split(' ')
        theta = float(theta)
        beta = float(beta)
        x = float(x)
        y = float(y)
        tractor_coords, trailer_coords = convertToPixels(x,y,theta,beta,h)
        map_str = lines[3:]
        map_int = []
        for line in map_str:
            map_int.append([int(x) for x in line.split(' ') if x != '\n'])

    #convert map_int to numpy array
    map_int = np.array(map_int)
    #change tractor and trailer coords in map to 1
    tractor_coords = np.array(tractor_coords)
    trailer_coords = np.array(trailer_coords)
    
    map_int[tuple(tractor_coords.T)] = 1
    map_int[tuple(trailer_coords.T)] = 1
    #display map_int as image
    plt.imshow(map_int)
    plt.show()


parseFiileandCreateArray('/home/naren/catkin_cl_rrt/tractor_trailer_motion_planning_16782/maps/map1.txt')