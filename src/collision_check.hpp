#include <iostream>
#define TRACTOR_WIDTH (5.0f)
#define L1 (10.0f)
#define L2 (10.0f)
#define M (1.0f)
#define D (0.5f)
#define MAP_WIDTH (100.0f)
#define MAP_HEIGHT (100.0f)
#define MAP_RESOLUTION (0.1f)
#define RECT_RESOLUTION (0.1f)
#include <Eigen.h>

class CollisionCheck {

    private: 
        double l_tractor = M + L1 + D/2;
        double l_trailer = L2 + D/2;
        Eigen::Matrix3d tractor_transform;
        Eigen::Matrix3d trailer_transform;

        inline void computeTransformMatrices (double theta, double beta, int x, int y) {
            
            trailer_transform << cos(theta), -sin(theta), x + L2/2*cos(theta),
            sin(theta), cos(theta), y + L2/2*sin(theta),
            0, 0, 1;
            tractor_transform << cos(theta+beta), -sin(theta+beta), x + L2*cos(theta) + l_tractor/2*cos(theta+beta),
            sin(theta+beta), cos(theta+beta), y + L2*sin(theta) + l_tractor/2*sin(theta+beta),
            0, 0, 1;
        }

        inline void computeCoords() {
            // tractor_coords<< -TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, -TRACTOR_WIDTH/2, 
            // l_tractor/2, l_tractor/2, -l_tractor/2, -l_tractor/2, 
            // 1, 1, 1, 1;

            // trailer_coords<< -TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, TRACTOR_WIDTH/2, -TRACTOR_WIDTH/2, 
            // (l_trailer)/2, (l_trailer)/2, -(l_trailer)/2, -(l_trailer)/2, 
            // 1, 1, 1, 1;

            int steps_w= (int) (TRACTOR_WIDTH/RECT_RESOLUTION)+1;
            int steps_l= (int) (l_tractor/RECT_RESOLUTION)+1;

            //define an eigen matrix to store the coordinates of the tractor_rect
            Eigen::Matrix3Xd tractor_rect_coords(3,(steps_w+1)*(steps_l+1));
            for (int i = 0; i <= steps_l; i++) {
                for (int j = 0; j < steps_w; j++) {
                    tractor_rect_coords(0,i*(steps_w+1)+j) = -TRACTOR_WIDTH/2 + j*float(TRACTOR_WIDTH/steps_w);
                    tractor_rect_coords(1,i*(steps_w+1)+j) = -l_tractor/2 + i*float(l_tractor/steps_l);
                    tractor_rect_coords(2,i*(steps_w+1)+j) = 1;
                }
            }

            steps_w= (int) (TRACTOR_WIDTH/RECT_RESOLUTION)+1;
            steps_l= (int) (l_trailer/RECT_RESOLUTION)+1;

            Eigen::Matrix3Xd trailer_rect_coords(3,(steps_w+1)*(steps_l+1));

            for (int i = 0; i <= steps_l; i++) {
                for (int j = 0; j < steps_w; j++) {
                    trailer_rect_coords(0,i*(steps_w+1)+j) = -TRACTOR_WIDTH/2 + j*float(TRACTOR_WIDTH/steps_w);
                    trailer_rect_coords(1,i*(steps_w+1)+j) = -l_trailer/2 + i*float(l_trailer/steps_l);
                    trailer_rect_coords(2,i*(steps_w+1)+j) = 1;
                }
            }

            tractor_coords = tractor_transform*tractor_coords;
            trailer_coords = trailer_transform*trailer_coords;
        }
       
        void converWorldXYtoGridXY(float x, float y, int &grid_x, int &grid_y) {
            // CONVERT world coordinates to grid coordinates 
            //map resolution is 0.1m
            //if x= 0.51m, grid_x = 6

            grid_x = (int) x/MAP_RESOLUTION;
            grid_y = (int) y/MAP_RESOLUTION;

        }

        int collision_check(int x, int y, double beta, double theta, double* world_map) {
            bool collided = false;
            computeTransformMatrices(theta, beta, x, y);
            computeCoords();
            
            while(!collided) {

                //first go over all coordinates of the tractor and check if they are in the obstacle space
                
                int grid_x, grid_y;
                for (int i = 0; i < tractor_coords.cols(); i++) {
                    converWorldXYtoGridXY(tractor_coords(0,i), tractor_coords(1,i), grid_x, grid_y);
                    //check if the grid_x and grid_y are within the map bounds and if the point is in the obstacle space
                    if (grid_x < 0 || grid_x >= MAP_WIDTH/MAP_RESOLUTION || grid_y < 0 || grid_y >= MAP_HEIGHT/MAP_RESOLUTION || world_map[grid_x + grid_y*MAP_WIDTH/MAP_RESOLUTION] == 1) {
                        collided = true;
                        cout<<"Collision detected at tractor point "<<tractor_coords(0,i)<<", "<<tractor_coords(1,i)<<endl;
                        break;
                    }
                }

                //now go over all coordinates of the trailer and check if they are in the obstacle space
                for (int i = 0; i < trailer_coords.cols(); i++) {
                    converWorldXYtoGridXY(trailer_coords(0,i), trailer_coords(1,i), grid_x, grid_y);
                    //check if the grid_x and grid_y are within the map bounds and if the point is in the obstacle space
                    if (grid_x < 0 || grid_x >= MAP_WIDTH/MAP_RESOLUTION || grid_y < 0 || grid_y >= MAP_HEIGHT/MAP_RESOLUTION || world_map[grid_x + grid_y*MAP_WIDTH/MAP_RESOLUTION] == 1) {
                        collided = true;
                        cout<<"Collision detected at trailer point "<<trailer_coords(0,i)<<", "<<trailer_coords(1,i)<<endl;
                        break;
                    }
                }
};

