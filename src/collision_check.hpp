#include <iostream>
#define TRACTOR_WIDTH (0.5)
#define L1 (1.0)
#define L2 (1.0)
#define BODY (0.5)
#define DIAMETER (0.5)
#define map_width (20)
#define map_height (20)
#define MAP_RESOLUTION (0.2)
#define RECT_RESOLUTION (0.05)
#include <eigen3/Eigen/Dense>

class CollisionCheck {

    private: 
        // double l_tractor = BODY + L1 + DIAMETER/2;
        // double l_trailer = L2 + DIAMETER/2;
        double l_tractor = 2.0;
        double l_trailer = 2.0;
        Eigen::Matrix3d tractor_transform;
        Eigen::Matrix3d trailer_transform;
        int steps_w= (int) (TRACTOR_WIDTH/RECT_RESOLUTION)+2;
        int steps_l_tractor= (int) (l_tractor/RECT_RESOLUTION)+2;
        int steps_l_trailer= (int) (l_trailer/RECT_RESOLUTION)+2;
        Eigen::MatrixXd tractor_coords = Eigen::MatrixXd::Zero(3, steps_w*steps_l_tractor);
        Eigen::MatrixXd trailer_coords = Eigen::MatrixXd::Zero(3, steps_w*steps_l_trailer);

        inline void computeTransformMatrices (double theta, double beta, int x, int y) {
            
            trailer_transform << cos(theta), -sin(theta), x + l_trailer/2*cos(theta),
            sin(theta), cos(theta), y + l_trailer/2*sin(theta),
            0, 0, 1;
            tractor_transform << cos(theta+beta), -sin(theta+beta), x + l_trailer*cos(theta) + l_tractor/2*cos(theta+beta),
            sin(theta+beta), cos(theta+beta), y + l_trailer*sin(theta) + l_tractor/2*sin(theta+beta),
            0, 0, 1;
        }

        inline void computeCoords() {


            Eigen::Matrix3Xd tractor_rect_coords(3,(steps_w)*(steps_l_tractor));
            for (int i = 0; i < steps_w; i++) {
                for (int j = 0; j < steps_l_tractor; j++) {
                    tractor_rect_coords(0,i*(steps_l_tractor)+j) = -l_tractor/2 + j*float(RECT_RESOLUTION);
                    tractor_rect_coords(1,i*(steps_l_tractor)+j) = -TRACTOR_WIDTH/2 + i*float(RECT_RESOLUTION);
                    tractor_rect_coords(2,i*(steps_l_tractor)+j) = 1;
                }
            }


            Eigen::Matrix3Xd trailer_rect_coords(3,(steps_w)*(steps_l_trailer));
            for (int i = 0; i < steps_w; i++) {
                for (int j = 0; j < steps_l_trailer; j++) {
                    trailer_rect_coords(0,i*(steps_l_trailer)+j) = -l_trailer/2 + j*float(RECT_RESOLUTION);
                    trailer_rect_coords(1,i*(steps_l_trailer)+j) = -TRACTOR_WIDTH/2 + i*float(RECT_RESOLUTION);
                    trailer_rect_coords(2,i*(steps_l_trailer)+j) = 1;
                }
            }

            std::cout<<"Tractor rect coords: "<<tractor_rect_coords.rows()<<"x"<<tractor_rect_coords.cols()<<std::endl;
            std::cout<<"Trailer rect coords: "<<trailer_rect_coords.rows()<<"x"<<trailer_rect_coords.cols()<<std::endl;
    
            tractor_coords = tractor_transform*tractor_rect_coords;
            trailer_coords = trailer_transform*trailer_rect_coords;

            // print dimensions of tractor_coords and trailer_coors
            std::cout<<"Tractor coords: "<<tractor_coords.rows()<<"x"<<tractor_coords.cols()<<std::endl;
            std::cout<<"Trailer coords: "<<trailer_coords.rows()<<"x"<<trailer_coords.cols()<<std::endl;
        }
       
        void converWorldXYtoGridXY(float x, float y, int &grid_x, int &grid_y) {
            // printf("x: %f, y: %f)", x, y);
            grid_x = (int) x/MAP_RESOLUTION;
            grid_y = (int) y/MAP_RESOLUTION;

        }
    public:
        int collision_check(float x, float y, float theta, float beta, double* world_map, int x_size, int y_size) {
            // print  and y_size
            std::cout<<"x_size: "<<x_size<<" y_size: "<<y_size<<std::endl;
            std::cout<< "map_width: "<<map_width<<" map_height: "<<map_height<<std::endl;
            
            assert(x_size == map_width/MAP_RESOLUTION && y_size == map_height/MAP_RESOLUTION);

            bool collided = false;
            computeTransformMatrices(theta, beta, x, y);
            computeCoords();

            int grid_x, grid_y;
            for (int i = 0; i < tractor_coords.cols(); i++) {
                std::cout<<"Tractor coords: "<<tractor_coords(0,i)<<" "<<tractor_coords(1,i)<<std::endl;
                converWorldXYtoGridXY(tractor_coords(0,i), tractor_coords(1,i), grid_x, grid_y);
                if (grid_x < 0 || grid_x >= x_size || grid_y < 0 || grid_y >= y_size || world_map[y_size-grid_y-1+grid_x*y_size] > 0) {
                    collided = true;
                    std::cout<<"Collision detected at tractor point "<<tractor_coords(0,i)<<", "<<tractor_coords(1,i)<<"\n";
                    break;
                }
            }

            for (int i = 0; i < trailer_coords.cols(); i++) {
                converWorldXYtoGridXY(trailer_coords(0,i), trailer_coords(1,i), grid_x, grid_y);
                if (grid_x < 0 || grid_x >= x_size || grid_y < 0 || grid_y >= y_size || world_map[y_size-grid_y-1+grid_x*y_size] >0) {
                    collided = true;
                    std::cout<<"Collision detected at trailer point "<<trailer_coords(0,i)<<", "<<trailer_coords(1,i)<<"\n";
                    break;
                }
            }
            return collided==true?0:1;
        }
};

