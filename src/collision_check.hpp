#include <Eigen.h>
#include <iostream>s
#define TRACTOR_WIDTH (5.0f)
#define L1 (10.0f)
#define L2 (10.0f)
#define M (1.0f)
#define D (0.5f)
#define MAP_WIDTH (100.0f)
#define MAP_HEIGHT (100.0f)
#define MAP_RESOLUTION (0.1f)
class CollisionCheck {

    private: 
        double l_tractor = M + L1 + D/2;
        double l_trailer = L2 + D/2;
        Eigen::Matrix3d tractor_transform;
        Eigen::Matrix3d trailer_transform;
        Eigen::Matrix3Xd tractor_coords(3,4);
        Eigen::Matrix3Xd trailer_coords(3,4);
        inline void computeTransformMatrices (double theta, double beta, int x, int y) {
            
            trailer_transform << cos(theta), -sin(theta), x + L2/2*cos(theta),
            sin(theta), cos(theta), y + L2/2*sin(theta),
            0, 0, 1;
            tractor_transform << cos(theta+beta), -sin(theta+beta), x + L2*cos(theta) + l_tractor/2*cos(theta+beta),
            sin(theta+beta), cos(theta+beta), y + L2*sin(theta) + l_tractor/2*sin(theta+beta),
            0, 0, 1;
        }
        inline void computeCoords() {
            // define a 3X4 matrix
            
            tractor_coords<< -TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, -TRACTOR_WIDTH/2, 
            l_tractor/2, l_tractor/2, -l_tractor/2, -l_tractor/2, 
            1, 1, 1, 1;

            trailer_coords<< -TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, -TRACTOR_WIDTH/2, 
            (l_trailer)/2, (l_trailer)/2, -(l_trailer)/2, -(l_trailer)/2, 
            1, 1, 1, 1;

            tractor_coords = tractor_transform*tractor_coords;
            trailer_coords = trailer_transform*trailer_coords;
        }

        
       
        void converWorldXYtoGridXY(float x, float y, int &grid_x, int &grid_y) {
            // CONVERT world coordinates to grid coordinates
            grid_x = (int) x*MAP_RESOLUTION/MAP_WIDTH;
            grid_y = (int) y*MAP_RESOLUTION/MAP_HEIGHT;
        }

        int collision_check(int x, int y, double beta, double theta, double* world_map) {
            bool collided = false;
            computeTransformMatrices(theta, beta, x, y);
            computeCoords();
            float corner_x = tractor_coords(0,0);
            float corner_y = tractor_coords(1,0);
            int grid_x, grid_y;
            converWorldXYtoGridXY(corner_x, corner_y, grid_x, grid_y);
            while(!collided) {
               
                if(grid_x < 0 || grid_x > MAP_WIDTH || grid_y < 0 || grid_y > MAP_HEIGHT || world_map[grid_x][grid_y] > 50) {
                    collided = true;
                }
                else {
                    // check the next co ordinate
                    corner_x += 0.1;
                    corner_y += 0.1;
                    converWorldXYtoGridXY(corner_x, corner_y, grid_x, grid_y);
                    
                }
            }
            // Check if the point is in the map
            if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT) {
                return 1;
            }
            // Check if the point is in the obstacle
            if (world_map[x + y*MAP_WIDTH] == 1) {
                return 1;
            }
            // Check if the point is in the tractor
            // float x1 = x - L1*cos(theta);
            // float y1 = y - L1*sin(theta);
            // float x2 = x + L2*cos(theta + beta);
            // float y2 = y + L2*sin(theta + beta);
            // float x3 = x + L2*cos(theta - beta);
            // float y3 = y + L2*sin(theta - beta);
            // float x4 = x - L1*cos(theta) + L2*cos(theta + beta);
            // float y4 = y - L1*sin(theta) + L2*sin(theta + beta);
            // float x5 = x - L1*cos(theta) + L2*cos(theta - beta);
            // float y5 = y - L1*sin(theta) + L2*sin(theta - beta);

            //take corner points from tractor_coords and trailer_coords

            // check if the points are in the map
            // check if the points are in the obstacle

            if (x1 < 0 || x1 >= MAP_WIDTH || y1 < 0 || y1 >= MAP_HEIGHT) {
                return 1;
            }
            if (x2 < 0 || x2 >= MAP_WIDTH || y2 < 0 || y2 >= MAP_HEIGHT) {
                return 1;
            }
            if (x3 < 0 || x3 >= MAP_WIDTH || y3 < 0 || y3 >= MAP_HEIGHT) {
                return 1;
            }
            if (x4 < 0 || x4 >= MAP_WIDTH || y4 < 0 || y4 >= MAP_HEIGHT) {
                return 1;
            }
            if (x5 < 0 || x5 >= MAP_WIDTH || y5 < 0 || y5 >= MAP_HEIGHT) {
                return 1;
            }
            if (world_map[x1 + y1*MAP_WIDTH] == 1) {
                return 1;
            }
            if (world_map[x2 + y2*MAP_WIDTH] == 1) {
                return 1;
            }
            if (world_map[x3 + y3*MAP_WIDTH] == 1) {
                return 1;
            } 
        }
};

