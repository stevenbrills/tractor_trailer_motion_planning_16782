#include "collision_check.hpp"
#include "utils.hpp"
int main() {
    double* world_map;
	int x_size, y_size;
    std::tie(world_map, x_size, y_size) = loadMap();
    CollisionCheck cc;
    cc.collision_check(0, 0, 0, 0, NULL);
    return 0;
}