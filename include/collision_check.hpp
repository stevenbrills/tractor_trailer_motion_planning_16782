#include <iostream>

#define TRACTOR_WIDTH (0.1)
#define L1 (0.5)
#define L2 (0.3)
#define BODY (0.0)
#define DIAMETER (0.0)
// #define map_width (100.0)
// #define map_height (100.0)
#define MAP_RESOLUTION (0.05)
#define RECT_RESOLUTION (0.02)
#include <eigen3/Eigen/Dense>

class CollisionCheck {

    private: 
        // double l_tractor = BODY + L1 + DIAMETER/2;
        // double l_trailer = L2 + DIAMETER/2;
        double l_tractor;
        double l_trailer;
        Eigen::Matrix3d tractor_transform;
        Eigen::Matrix3d trailer_transform;
        Eigen::Matrix3Xd tractor_coords;
        Eigen::Matrix3Xd trailer_coords;

        inline void computeTransformMatrices (double theta, double beta, int x, int y) {
            
            trailer_transform << cos(theta), -sin(theta), x + l_trailer/2*cos(theta),
            sin(theta), cos(theta), y + l_trailer/2*sin(theta),
            0, 0, 1;
            tractor_transform << cos(theta+beta), -sin(theta+beta), x + l_trailer*cos(theta) + l_tractor/2*cos(theta+beta),
            sin(theta+beta), cos(theta+beta), y + l_trailer*sin(theta) + l_tractor/2*sin(theta+beta),
            0, 0, 1;

        }

        inline void computeCoords() {

            int steps_w= (int) (TRACTOR_WIDTH/RECT_RESOLUTION)+1;
            int steps_l= (int) (l_tractor/RECT_RESOLUTION)+1;

            //define an eigen matrix to store the coordinates of the tractor_rect
            Eigen::Matrix3Xd tractor_rect_coords(3,(steps_w+1)*(steps_l+1));
            for (int i = 0; i <= steps_l; i++) {
                for (int j = 0; j < steps_w; j++) {
                    tractor_rect_coords(0,i*(steps_w+1)+j) = l_tractor/2 - i*float(l_tractor/steps_l);
                    tractor_rect_coords(1,i*(steps_w+1)+j) = TRACTOR_WIDTH/2 - j*float(TRACTOR_WIDTH/steps_w);
                    tractor_rect_coords(2,i*(steps_w+1)+j) = 1;
                }
            }

            steps_w= (int) (TRACTOR_WIDTH/RECT_RESOLUTION)+1;
            steps_l= (int) (l_trailer/RECT_RESOLUTION)+1;

            Eigen::Matrix3Xd trailer_rect_coords(3,(steps_w+1)*(steps_l+1));

            for (int i = 0; i <= steps_l; i++) {
                for (int j = 0; j < steps_w; j++) {
                    trailer_rect_coords(0,i*(steps_w+1)+j) = l_trailer/2 - i*float(l_trailer/steps_l);
                    trailer_rect_coords(1,i*(steps_w+1)+j) = TRACTOR_WIDTH/2 - j*float(TRACTOR_WIDTH/steps_w);
                    trailer_rect_coords(2,i*(steps_w+1)+j) = 1;
                }
            }
            
            // std::cout << "Tractor TF: " << tractor_transform.rows() << "X" << tractor_transform.cols() << std::endl;
            // std::cout << "Tractor Coords TF: " << tractor_rect_coords.rows() << "X" << tractor_rect_coords.cols() << std::endl;
            // std::cout << "Trailer TF: " << trailer_transform.rows() << "X" << trailer_transform.cols() << std::endl;
            // std::cout << "Trailer Coords TF: " << trailer_rect_coords.rows() << "X" << trailer_rect_coords.cols() << std::endl;


            tractor_coords = tractor_transform*tractor_rect_coords;
            trailer_coords = trailer_transform*trailer_rect_coords;

            // std::cout << "Finished mat mul" << std::endl;
        }
       
        void converWorldXYtoGridXY(float x, float y, int &grid_x, int &grid_y) {
            // CONVERT world coordinates to grid coordinates 
            //map resolution is 0.1m
            //if x= 0.51m, grid_x = 6

            grid_x = (int) x/MAP_RESOLUTION;
            grid_y = (int) y/MAP_RESOLUTION;

        }
    public:

        CollisionCheck(
            const double& tractor_wheelbase,
            const double& trailer_wheelbase,
            const double& tractor_hitch_offset
        ){
            this->l_tractor = tractor_wheelbase + tractor_hitch_offset;
            this->l_trailer = trailer_wheelbase;
        }
        int collision_check(double x, double y, double beta, double theta, double* world_map, double map_height, double map_width) {
            
            bool collided = false;
            computeTransformMatrices(theta, beta, x, y);
            computeCoords();
            // std::cout<<"Tractor coords: "<<tractor_coords<<"\n";
            // std::cout<<"Trailer coords: "<<trailer_coords<<"\n";

            //first go over all coordinates of the tractor and check if they are in the obstacle space
            int grid_x, grid_y;
            for (int i = 0; i < tractor_coords.cols(); i++) {
                converWorldXYtoGridXY(tractor_coords(0,i), tractor_coords(1,i), grid_x, grid_y);
                //check if the grid_x and grid_y are within the map bounds and if the point is in the obstacle space
                // std::cout << "Grid X: " << grid_x << "     Grid Y: " << grid_y << std::endl;
                // std::cout << "World map size: " << sizeof(world_map)/sizeof(world_map[0]) << std::endl;
                // std::cout << "Going to access elements index: " << grid_x + grid_y*(int)(map_width/MAP_RESOLUTION) << std::endl;
                // std::cout << "World value at grid_x and grid_y is: " << world_map[grid_x + grid_y*(int)(map_width/MAP_RESOLUTION)] << std::endl;
                if (grid_x < 0 || grid_x >= (int)(map_width/MAP_RESOLUTION) || grid_y < 0 || grid_y >= (int)(map_height/MAP_RESOLUTION) || world_map[grid_x + grid_y*(int)(map_width/MAP_RESOLUTION)] > 0) {
                    collided = true;
                    std::cout<<"Collision detected at tractor point "<<tractor_coords(0,i)<<", "<<tractor_coords(1,i)<<"\n";
                    break;
                }
            }

            //now go over all coordinates of the trailer and check if they are in the obstacle space
            for (int i = 0; i < trailer_coords.cols(); i++) {
                converWorldXYtoGridXY(trailer_coords(0,i), trailer_coords(1,i), grid_x, grid_y);
                if (grid_x < 0 || grid_x >= (int)(map_width/MAP_RESOLUTION) || grid_y < 0 || grid_y >= (int)(map_height/MAP_RESOLUTION) || world_map[grid_x + grid_y*(int)(map_width/MAP_RESOLUTION)] >0) {
                    collided = true;
                    std::cout<<"Collision detected at tractor point "<<tractor_coords(0,i)<<", "<<tractor_coords(1,i)<<"\n";
                    break;
                }
            }
            return collided==true?1:0;
        }
};