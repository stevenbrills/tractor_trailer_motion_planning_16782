import numpy as np
import math
from icecream import ic
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec
from numpy import loadtxt

class collision_check():
    def __init__(self):
        self.L2 = 1.0
        self.L1 = 1.0
        self.DIAMETER = 0.5 
        self.BODY= 0.5
        # self.l_tractor = self.BODY + self.L1 + self.DIAMETER/2
        # self.l_trailer = self.L2 + self.DIAMETER/2
        self.l_tractor = 2
        self.l_trailer = 2
        self.TRACTOR_WIDTH = 0.5
        self.RECT_RESOLUTION = 0.05
        self.MAP_RESOLUTION = 0.2
        self.map_width = 20
        self.map_height = 20
        self.world_map = loadtxt('./map1.txt')
        assert(self.world_map.shape[0] == self.map_height/self.MAP_RESOLUTION)
        assert(self.world_map.shape[1] == self.map_width/self.MAP_RESOLUTION)
        self.tractor_coords = np.zeros((3, 1))
        self.trailer_coords = np.zeros((3, 1))
        self.tractor_transform = np.zeros((3, 3))
        self.trailer_transform = np.zeros((3, 3))
        self.tractor_rect_coords = np.zeros((3, 1))
        self.trailer_rect_coords = np.zeros((3, 1))

    def computeTransformMatrices(self, theta, beta, x, y):
        # theta = theta*math.pi/180
        # beta = beta*math.pi/180

        self.trailer_transform = np.array([[math.cos(theta), -math.sin(theta), x + self.l_trailer/2*math.cos(theta)],
                                           [math.sin(theta), math.cos(theta), y + self.l_trailer/2*math.sin(theta)],
                                           [0, 0, 1]])
        self.tractor_transform = np.array([[math.cos(theta+beta), -math.sin(theta+beta), x + self.l_trailer*math.cos(theta) + self.l_tractor/2*math.cos(theta+beta)],
                                           [math.sin(theta+beta), math.cos(theta+beta), y + self.l_trailer*math.sin(theta) + self.l_tractor/2*math.sin(theta+beta)],
                                           [0, 0, 1]])
    
    def computeCoords(self):
        steps_w = int(self.TRACTOR_WIDTH/self.RECT_RESOLUTION) + 2
        steps_l = int(self.l_tractor/self.RECT_RESOLUTION) + 2

        ic(self.l_tractor)
        ic(steps_w)
        ic(steps_l)
        tractor_rect_coords = np.zeros((3, (steps_w)*(steps_l)))
        for i in range(steps_w):
            for j in range(steps_l):
                tractor_rect_coords[0, i*(steps_l)+j] = -self.l_tractor/2 + j*self.RECT_RESOLUTION
                tractor_rect_coords[1, i*(steps_l)+j] = -self.TRACTOR_WIDTH/2 + i*self.RECT_RESOLUTION
                tractor_rect_coords[2, i*(steps_l)+j] = 1
        
        steps_w = int(self.TRACTOR_WIDTH/self.RECT_RESOLUTION) + 2
        steps_l = int(self.l_trailer/self.RECT_RESOLUTION) + 2
        trailer_rect_coords = np.zeros((3, (steps_w)*(steps_l)))
        ic(self.l_trailer)
        ic(steps_w)
        ic(steps_l)
        for i in range(steps_w):
            for j in range(steps_l):
                
                trailer_rect_coords[0, i*(steps_l)+j] = -self.l_trailer/2 + j*self.RECT_RESOLUTION
                trailer_rect_coords[1, i*(steps_l)+j] = -self.TRACTOR_WIDTH/2 + i*self.RECT_RESOLUTION
                trailer_rect_coords[2, i*(steps_l)+j] = 1
               
        ic(self.tractor_transform)
        ic(self.trailer_transform)

        self.tractor_coords = np.matmul(self.tractor_transform, tractor_rect_coords)
        self.trailer_coords = np.matmul(self.trailer_transform, trailer_rect_coords)

        ic(self.tractor_coords.shape)
        ic(self.trailer_coords.shape)

        # two plots in one window
        # fig = plt.figure(figsize=(10, 5))
        # gs = GridSpec(1, 2, figure=fig)
        # ax1 = fig.add_subplot(gs[0, 0])
        # ax2 = fig.add_subplot(gs[0, 1])
        # ax1.set_aspect('equal')
        # ax2.set_aspect('equal')
        # ax2.plot(self.tractor_coords[0, :], self.tractor_coords[1, :], 'b.')
        # ax1.plot(tractor_rect_coords[0, :], tractor_rect_coords[1, :], 'r.')

        fig = plt.figure(figsize=(9,9))
        gs = GridSpec(nrows=2, ncols=2)
        ax0 = fig.add_subplot(gs[0, 0])
        ax1 = fig.add_subplot(gs[1, 0])
        ax0.set_xlim(-self.l_tractor/2-self.RECT_RESOLUTION*2, self.l_tractor/2+self.RECT_RESOLUTION*2)
        ax0.set_ylim(-self.TRACTOR_WIDTH/2-self.RECT_RESOLUTION*2, self.TRACTOR_WIDTH/2+self.RECT_RESOLUTION*2)
        ax1.set_xlim(-self.l_trailer/2-self.RECT_RESOLUTION*2, self.l_trailer/2+self.RECT_RESOLUTION*2)
        ax1.set_ylim(-self.TRACTOR_WIDTH/2-self.RECT_RESOLUTION*2, self.TRACTOR_WIDTH/2+self.RECT_RESOLUTION*2)
        ax0.set_aspect('equal')
        ax1.set_aspect('equal')

        ax0.plot(tractor_rect_coords[0, :], tractor_rect_coords[1, :], 'rs')
        ax1.plot(trailer_rect_coords[0, :], trailer_rect_coords[1, :], 'bs')
        ax2 = fig.add_subplot(gs[:, 1])
        ax2.set_aspect('equal')
        ax2.set_xlim(0, self.map_width)
        ax2.set_ylim(0, self.map_height)
        ax2.plot(self.tractor_coords[0, :], self.tractor_coords[1, :], 'rs')
        ax2.plot(self.trailer_coords[0, :], self.trailer_coords[1, :], 'bs')
        plt.savefig('test.png')

    def checkCollision(self):
        grid_height= int(self.map_height/self.MAP_RESOLUTION)
        grid_width = int(self.map_width/self.MAP_RESOLUTION)
        collided = False
        
        #first go over all coordinates of the tractor and check if they are in the obstacle space 
        for i in range(self.tractor_coords.shape[1]):
            grid_x_tractor = int(self.tractor_coords[0, i]/self.MAP_RESOLUTION)
            grid_y_tractor = int(self.tractor_coords[1, i]/self.MAP_RESOLUTION)
            # check if the grid_x_tractor and grid_y_tractor are within the map bounds and if the point is in the obstacle space
            if grid_x_tractor < 0 or grid_x_tractor >= int(self.map_width/self.MAP_RESOLUTION) or grid_y_tractor < 0 or grid_y_tractor >= int(self.map_height/self.MAP_RESOLUTION) or self.world_map[grid_height-grid_y_tractor-1, grid_x_tractor] > 0:
                collided = True
                print("Collision detected at tractor point ", self.tractor_coords[0, i], ", ", self.tractor_coords[1, i])
                break

        #now go over all coordinates of the trailer and check if they are in the obstacle space
        for i in range(self.trailer_coords.shape[1]):
            grid_x_trailer = int(self.trailer_coords[0, i]/self.MAP_RESOLUTION)
            grid_y_trailer = int(self.trailer_coords[1, i]/self.MAP_RESOLUTION)
            if grid_x_trailer < 0 or grid_x_trailer >= int(self.map_width/self.MAP_RESOLUTION) or grid_y_trailer < 0 or grid_y_trailer >= int(self.map_height/self.MAP_RESOLUTION) or self.world_map[grid_height-grid_y_trailer-1, grid_x_trailer] > 0:
                collided = True
                print("Collision detected at trailer point ", self.trailer_coords[0, i], ", ", self.trailer_coords[1, i])
                break
        return 0 if collided else print("No collision detected")
       
    
    def visualise(self):
        grid_height= int(self.map_height/self.MAP_RESOLUTION)
        for i in range(self.tractor_coords.shape[1]):
            grid_x_tractor = int(self.tractor_coords[0, i]/self.MAP_RESOLUTION)
            grid_y_tractor = int(self.tractor_coords[1, i]/self.MAP_RESOLUTION)
            self.world_map[grid_height-grid_y_tractor-1, grid_x_tractor] = 1
       
        for i in range(self.trailer_coords.shape[1]):
            grid_x_trailer = int(self.trailer_coords[0, i]/self.MAP_RESOLUTION)
            grid_y_trailer = int(self.trailer_coords[1, i]/self.MAP_RESOLUTION)
            self.world_map[grid_height-grid_y_trailer-1, grid_x_trailer] = 1
        fig2 = plt.figure(figsize=(9,9))
        plt.imshow(self.world_map, interpolation='nearest')
        plt.savefig('test2.png')

    
if __name__=='__main__':
    cc = collision_check()
    cc.computeTransformMatrices(0.785, 0.785, 5,5)
    cc.computeCoords()
    cc.checkCollision()
    cc.visualise()

   