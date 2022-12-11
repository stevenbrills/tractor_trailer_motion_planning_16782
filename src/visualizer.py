import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from icecream import ic
from matplotlib.animation import FuncAnimation, PillowWriter  # For gif
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull

import pdb # Use this for debugging python!
matplotlib.use('Agg')
import argparse
import re
#define TRACTOR_WIDTH (2.0)
#define L1 (5.0)
#define L2 (5.0)
#define BODY (1.0)
#define DIAMETER (0.5)
#define MAP_RESOLUTION (0.1)
#define RECT_RESOLUTION (0.1)
TRACTOR_WIDTH = 2.0
L1 = 0.3
L2 = 0.8
BODY = 0.2
DIAMETER = 0.5
MAP_RESOLUTION = 0.1
RECT_RESOLUTION = 0.1
DOWNSAMPLE_FACTOR = 500

def readMap(mapfile):
    """ Input: mapfile path
    Output: 2D numpy array with binary values based on the map
    """
    with open(mapfile) as f:
        line = f.readline()  # e.g. "height 50"
        height = int(line.split(' ')[1])
        line = f.readline()  # e.g. "width 50"
        width = int(line.split(' ')[1])
        mapdata = np.array([line.rstrip().split(" ") for line in f])

    mapdata.reshape((width,height))
    mapdata = mapdata.astype(int)
    return mapdata


def calculateEndPoints(x,y,theta,beta):
    tract_length = L1+BODY
    tractor_coords = []
    trailer_coords = []
    theta = float(theta)
    beta = float(beta)
    trailer_coords.append([x,y])
    x1 = (x+L2*np.cos(theta))
    y1 = (y+L2*np.sin(theta))
    trailer_coords.append([x1,y1])

    tractor_coords.append([x1,y1])
    x2 = (x+L2*np.cos(theta)+tract_length*np.cos(theta+beta))
    y2 = (y+L2*np.sin(theta)+tract_length*np.sin(theta+beta))
    tractor_coords.append([x2,y2])

    # ic(trailer_coords)
    # ic(tractor_coords)  
    return tractor_coords, trailer_coords
    
def createSingleFrame(i, tractor_coords, trailer_coords, includePrevious, ax, polygons, piecewise_llinear_paths, L):

    if not includePrevious:
        plt.clf()
    tractorData = tractor_coords[i]
    trailerData = trailer_coords[i]
    artists = []
    p1 = [tractorData[0][0],tractorData[0][1]]
    p2 = [tractorData[-1][0],tractorData[-1][1]]
    x_trac, y_tract = [p1[0], p2[0]], [p1[1], p2[1]]
    # plt.plot(x, y, color = 'red', linewidth=15)
    p1 = [trailerData[0][0],trailerData[0][1]]
    p2 = [trailerData[-1][0],trailerData[-1][1]]
    x_trailer, y_trailer = [p1[0], p2[0]], [p1[1], p2[1]]
    plt.xlim(-2,105)
    plt.ylim(-2,105)
    plt.axis('off')
    artists.append(plt.plot(x_trailer, y_trailer, color = 'blue', linewidth=8))
    artists.append(plt.plot(x_trac, y_tract, color = 'red', linewidth=8))
    for p in polygons:
        p=np.array(p)
        hull = ConvexHull(p)

        cent = np.mean(p, 0)
        pts = []
        for pt in p[hull.simplices]:
            pts.append(pt[0].tolist())
            pts.append(pt[1].tolist())

        pts.sort(key=lambda p: np.arctan2(p[1] - cent[1],
                                        p[0] - cent[0]))
        pts = pts[0::2]  # Deleting duplicates
        pts.insert(len(pts), pts[0])
        k = 1.1
        color = 'black'
        poly = Polygon(k*(np.array(pts)- cent) + cent,facecolor=color, alpha=1)
    
        patch = poly
        artists.append(plt.gca().add_patch(patch))
    for k in range(1,len(piecewise_llinear_paths)):
        p1, p2 = [piecewise_llinear_paths[k-1][0], piecewise_llinear_paths[k][0]], [piecewise_llinear_paths[k-1][1], piecewise_llinear_paths[k][1]]
        if k ==1:
            artists.append(plt.plot(p1, p2, color = 'green', linewidth=2, linestyle='dashed', label = 'Piecewise Linear Path'))
        else:
            artists.append(plt.plot(p1, p2, color = 'green', linewidth=2, linestyle='dashed'))
    artists.append(plt.legend(loc='upper right'))   
    print("i value: ", i)
    return artists

def parseFiileandCreateArray():
    parser = argparse.ArgumentParser()
    parser.add_argument("--filename", help="file for trajectory", type=str, default="../output/PlannedTrajectory.txt")
    parser.add_argument("--map_file", help="filepath with solution", type=str, default="../input/mapinfo.txt")
    parser.add_argument("--gifFilepath", help="filepath for gif", type=str, default="../ouputs/planned_goal.gif", required=False)
    parser.add_argument("--fps", help="frames per second", type=int, default=20, required=False)
    parser.add_argument("--incPrev", help="include previous poses (1), else don't (0). Note this slows gif creation speed",
                                        type=int, default=0, required=False)
    
    args = parser.parse_args()
    assert(args.gifFilepath.endswith(".gif"))  # Make sure it ends with a .gif extension
    assert(args.incPrev == 0 or args.incPrev == 1)  # 0 is don't include, 1 is include
    
    polygons = []
 
    # Open the file for reading
    with open(args.map_file, 'r') as f:
        # Read the file line by line
        for line in f:
            # Split the line into words
            words = line.split()
            ic(words)

            if words == []:
                break

            if words[0] == 'obstacles':
                
                line = next(f)
                words=line.split()
                # Read the vertices until we reach the next keyword
                while words[0] != 'height':

                    words = line.split(" ")

                    p = []
                    for v in words:
                        sub1 = "("
                        sub2 = ","
                        s=str(re.escape(sub1))
                        e=str(re.escape(sub2))
                        # printing result
                        x_coord=re.findall(s+"(.*)"+e,v)[0]
                        x = float(x_coord)
                        sub3 = ")"
                        sub3_esc = str(re.escape(sub3))
                        y_coord = re.findall(e+"(.*)"+sub3_esc,v)[0]
                        y = float(y_coord)
                        p.append([x,y])
                    polygons.append(p)
                    
                    line = next(f)
                    words=line.split()
                    if words[0] == 'height': 
                        break
       
    path_reversible = []
    tractor_coords = []
    trailer_coords = []
    i = 0
    found = False
    with open(args.filename) as f:
        lines = f.readlines()
        for line in lines:
            if line.strip()== 'Trajectory':
                found = True
                continue
            if not found:
                continue
            if i%DOWNSAMPLE_FACTOR == 0:
                print("Processing line: ", i)
                x,y,theta,beta,x2,y2,alpha = line.split(' ')
                x = float(x)
                y = float(y)
                theta = float(theta)
                beta = float(beta)
                tractor_coords_temp, trailer_coords_temp = calculateEndPoints(x,y,theta,beta)
                tractor_coords.append(tractor_coords_temp)
                trailer_coords.append(trailer_coords_temp)
            i+=1
    #change tractor and trailer coords in map to 1
    i = 0
    points = []
    piecewise_llinear_paths = []
    found = False
    with open(args.filename) as f:
        lines = f.readlines()
        for line in lines:
            if line.strip() == 'Piecewise Linear Path':  # Or whatever test is needed
                found = True
                continue
    # Reads text until the end of the block:
            if line.strip() == 'Trajectory':
                break
            if not found:
                continue
            x,y = float(line.split(' ')[0]), float(line.split(' ')[1])
            piecewise_llinear_paths.append([x,y])
    print(piecewise_llinear_paths)
    fig,ax = plt.subplots()
    numFrames = len(tractor_coords)
    print("numFrames: ", numFrames)
    L = plt.legend(loc='upper right', shadow=True, fontsize='large')
    ani = FuncAnimation(fig, createSingleFrame, repeat=False,
        frames=numFrames, fargs=[ tractor_coords, trailer_coords, args.incPrev, ax, polygons, piecewise_llinear_paths, L])    
    ani.save(args.gifFilepath, dpi=100, writer=PillowWriter(fps=args.fps))
    print("Saved gif to: ", args.gifFilepath)

if __name__ == "__main__":
    parseFiileandCreateArray()
