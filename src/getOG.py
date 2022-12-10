from icecream import ic
import argparse
import re
from PIL import Image
import numpy as np
import cv2
    
def readVertices():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map_file", help="filepath with solution", type=str, default="vertices.txt")
    args = parser.parse_args()

    polygons = []
    lin_num = 0
    with open(args.map_file) as f:
        lines = f.readlines()
        for line in lines:
            if lin_num == 0:
                vertices = line.split(" ")
                p = []
                for v in vertices:
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
                p = np.array(p)
                polygons.append(p)
                lin_num += 1

    return polygons

def img_toGrid(img,height,width,res):
    img = Image.open(img).convert('L')

    # Define the width and height of the occupancy grid
    width = int(width)
    height =int(height)

    # Resize the image to the desired width and height
    img = img.resize((width, height), resample=Image.Resampling.NEAREST)

    # Convert the image to a numpy array
    grid = np.array(img)

    ic(grid.shape)
    # Convert the values in the array to 0 or 1
    grid[grid < 128] = 1
    grid[grid >= 128] = 0

    # Save the occupancy grid as a text file
    np.savetxt('grid.txt', grid, fmt='%d')

if __name__ == "__main__":

    #basically image is the 
    res=0.05
    world_height=100 # in meters
    world_width=100 # in meters


    vertices = readVertices()[0]

    vertices=vertices/res

    # Define the width and height of the image
    width = int(world_width*(1.0/res))
    height = int(world_height*(1.0/res))


    # Create an empty image with the desired width and height
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Fill the image with white
    img.fill(255)

    # Create a list of the polygon vertices as integers
    int_vertices = vertices.astype(int)

    # Create the polygon
    polygon = cv2.convexHull(int_vertices)

    cv2.fillConvexPoly(img, polygon, (0, 0, 0))

    img = np.flipud(img)
    # Save the image to a file
    cv2.imwrite('polygon.png', img)

    # open the images and save this image as an array in a text file
    # np.array(img).tofile('polygon.txt')

    img_toGrid('polygon.png',height-1,width-1,res)
   

