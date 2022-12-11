from icecream import ic
import argparse
import re
from PIL import Image
import numpy as np
import cv2
    
def read_file(file_name):
    # Initialize variables for the obstacles, height, width, and resolution
    obstacles = []
    height = None
    width = None
    resolution = None

    # Open the file for reading
    with open(file_name, 'r') as f:
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
                    obstacles.append(p)
                    
                    line = next(f)
                    words=line.split()
                    if words[0] == 'height': 
                        height = float(words[1])
            
            # If the first word is 'width', this is the width of the grid
            elif words[0] == 'width':
                # Parse the width from the second word
                width = float(words[1])

            # If the first word is 'resolution', this is the resolution of the grid
            elif words[0] == 'resolution':
                # Parse the resolution from the second word
                resolution = float(words[1])

    # Return the obstacles, height, width, and resolution as a tuple
    return obstacles, height, width, resolution


def img_toGrid(img,height,width):
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
    np.savetxt('../maps/map2.txt', grid, fmt='%d')

if __name__ == "__main__":
    vertices_list, world_height,world_width, res = read_file('../input/mapinfo.txt')
   
    # Define the width and height of the image
    width = int(world_width*(1.0/res))
    height = int(world_height*(1.0/res))

    # Create an empty image with the desired width and height
    img = np.zeros((height, width, 3), dtype=np.uint8)

    # Fill the image with white
    img.fill(255)

    # Create a list of the polygon vertices as integers
    for vertices in vertices_list:
        print(vertices)
        vertices = np.array(vertices)
        vertices=vertices/res
        int_vertices = vertices.astype(int)
        # Create the polygon
        polygon = cv2.convexHull(int_vertices)
        cv2.fillConvexPoly(img, polygon, (0, 0, 0))

    img = np.flipud(img)
    # Save the image to a file
    cv2.imwrite('polygon.png', img)

    img_toGrid('polygon.png',height,width)
   

