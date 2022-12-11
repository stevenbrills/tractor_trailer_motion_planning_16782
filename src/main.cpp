#include "cc.hpp"
#include "utils.hpp"

int main() {
    double* world_map;
	int x_size, y_size;
    std::tie(world_map, x_size, y_size) = loadMap("/home/suraj/tractor_trailer_motion_planning_16782/maps/map2.txt");
    // x_size = x_size*0.05;
    // y_size = y_size*0.05;
    // std::cout << "Size of world map in bytes: " << sizeof(world_map[1161000]) << std::endl;
    CollisionCheck cc(0.3, 0.8, 0.2);
    // std::cout << cc.collision_check(50.85, 70.8, 0.52, 0.52, world_map, x_size, y_size); // Supposed to collide in map2
    std::cout << "Trailer in collision? " << cc.collision_check(50.85, 20.9, 0.52, 0.52, world_map, x_size, y_size) << std::endl;  // Should not collide in map2
    // std::cout << cc.collision_check(100, 100, 0.52, 0.52, world_map, x_size, y_size); // Supposed to collide in map2
    return 0;
}