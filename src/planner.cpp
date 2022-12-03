#include "planner.h"
#include <vector>
#include <random>
#include <set>

double map_x = 100;
double map_y = 100;


std::vector<double> sample_control_point(
    std::vector<double> random_sample
){
    // Initialize mersenne twister object
    std::mt19937 mt(std::random_device{}());

    std::uniform_int_distribution<float> uniform_dist(0, 100);

    random_sample[0] = uniform_dist(mt);
    random_sample[1] = uniform_dist(mt);

}

std::vector<std::vector<double>> planner(
    const std::vector<double> q_init,
    const std::vector<double> q_goal
){

    // Number of sampling attempts max
    int sampling_trials = 5000;

    // Create the first node object which pairs q_init to initial control point
    Node* InitialNode = new Node;
    InitialNode->q = q_init;
    InitialNode->control_input[0] = q_init[0];
    InitialNode->control_input[1] = q_init[1];
    InitialNode->cost2cum = 0; 
    InitialNode->parent_node = std::nullptr;

    // Loop for sampling_trials number of times
    for(int i=0; i<sampling_trials; i++){

        std::vector<double> random_control_sample(2,0);
        sample_control_point(random_control_sample);

        // Store tree in unordered set
        unordered_set<Node*, GroundedConditionHasher, GroundedConditionComparator> tree;

        // Find nearest neighbor using euclidean distance
        for ();

        // Connect sample to control point of nearest neighbor

        // Expand the state/node

            // Call the collision-checker function

                // If collision free, add edge

                // Else resample



    }


}