#include "collision_check.hpp"
#include "utils.hpp"
int main() {
    double* world_map;
	int x_size, y_size;
    std::tie(world_map, x_size, y_size) = loadMap("map1_cpp.txt");
    CollisionCheck cc;
    std::cout<<"Collision value: "<<cc.collision_check(5.0, 5.0, 0.785, 0.785, world_map, x_size, y_size);
    return 0;
}