#ifndef UTILS_PLANNER_HEADER_GUARD
#define UTILS_PLANNER_HEADER_GUARD

#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	// std::cout << "In load map function" << std::endl;
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	// std::cout << "Height read: " << height << std::endl;
	// std::cout << "Width read: " << width << std::endl;

	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];
	// std::cout << "In load map, world_map size: " << sizeof(map[1161000]) << std::endl;

	int while_counter = 0;

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				while_counter++;
				// std::cout << (fscanf(f, "%c", &c)) << std::endl;
				if (fscanf(f, "%c", &c) != 1) {
					std::cout << while_counter << std::endl;
					// std::cout << (fscanf(f, "%c", &c) != 1) << std::endl;
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

#endif