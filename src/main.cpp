#include "collision_check.hpp"
#include "utils.hpp"
int main() {
    double* world_map;
	int x_size, y_size;
    std::tie(world_map, x_size, y_size) = loadMap("/home/naren/catkin_cl_rrt/tractor_trailer_motion_planning_16782/src/map1.txt");
    CollisionCheck cc;
    cc.collision_check(0, 0, 0, 0, world_map,x_size, y_size);
    return 0;
}