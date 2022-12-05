#include "planner.h"
#include <vector>
#include <random>
#include <unordered_set>
#include <queue>
#include <algorithm> 
#include <iostream>
#include <fstream>
#include <stdexcept>

#define SAMPLING_TRIALS 100
double map_side = 105;
// double map_y = 105;


static void sample_control_point(
    std::vector<double>& random_sample
){
    // Initialize mersenne twister object
    std::mt19937 mt(std::random_device{}());

    if(!(random_sample.size()==2)){
        throw std::runtime_error("Size of the vector for control sample is incorrect.");
    }

    std::uniform_real_distribution<float> uniform_dist(0, map_side);

    random_sample[0] = uniform_dist(mt);
    random_sample[1] = uniform_dist(mt);

}

double get_euclidean_distance(
    const std::vector<double>& p1,
    const std::vector<double>& p2
){
    return sqrt(pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2));
}

bool get_direction(
    const Node* node,
    const std::vector<double>& control_sample
){

    // This function returns false if the control sample is behind the rear axle
    // and returns true if the control sample is infront of the rear axle.

    float m = tan(node->q[2] - M_PI_2);
    float c = node->q[1] - (m*node->q[0]);

    float y = m*control_sample[0] + c;

    std::cout << "Computed value of y: " << y << std::endl;
    std::cout << "Control sample y: " << control_sample[0] << " "<< control_sample[1] << std::endl;
    if(std::isnan(y)){
        std::cout<<" y is nan"<<std::endl;
        return true;
    }
    if(y>=control_sample[1]){
        return false;
    }

    return true;
}

Node* pick_random_node_from_tree(
    const std::unordered_set<Node*, NodePtrHasher, NodePtrComparator>& tree
){
    std::mt19937 mt(std::random_device{}());
    std::uniform_int_distribution<int> uniform_dist(0, tree.size()-1);

    auto it = tree.begin();
    std::advance(it, uniform_dist(mt));

    return (*it);
}

std::pair<Node*, bool> find_nearest_neighbor(
    const std::unordered_set<Node*, NodePtrHasher, NodePtrComparator>& tree,
    const std::vector<double>& control_sample
){
    double lowest_score = sqrt(pow(((*(tree.begin()))->control_input[0] - control_sample[0]),2) + 
    pow(((*(tree.begin()))->control_input[1] - control_sample[1]),2));

    std::pair<Node*, bool> result;

    Node* nearest_node;

    for(Node* node : tree){

        if(get_euclidean_distance(control_sample, (*node).control_input) <= lowest_score){

            if(get_direction(node, control_sample)){

                std::vector<double> tractor_axle_center(2,0);
                tractor_axle_center.push_back((*node).q[4]);
                tractor_axle_center.push_back((*node).q[5]);
                if(get_euclidean_distance(control_sample, tractor_axle_center)>=FORWARD_LOOKAHEAD_DISTANCE){
                    lowest_score  = get_euclidean_distance(control_sample, (*node).control_input);
                    nearest_node = node;
                }

            }
            else{
                std::vector<double> trailer_axle_center(2,0);
                trailer_axle_center.push_back((*node).q[0]);
                trailer_axle_center.push_back((*node).q[1]);
                if(get_euclidean_distance(control_sample, trailer_axle_center)>=BACKWARD_LOOKAHEAD_DISTANCE){
                    lowest_score  = get_euclidean_distance(control_sample, (*node).control_input);
                    nearest_node = node;
                }
            }
        }
    }

    result.first = nearest_node;
    result.second = get_direction(nearest_node, control_sample);
    std::cout<<"exiting nearest neighbor"<<std::endl;
    return result;
}

std::vector<std::vector<double>> planner(
    const std::vector<double> q_init,
    const std::vector<double> q_goal
){

    // Number of sampling attempts max
    int sampling_trials = SAMPLING_TRIALS;

    // Segment tracking direction boolean vector
    std::vector<bool> tracking_direction;

    // Create the first node object which pairs q_init to initial control point
    Node* InitialNode = new Node;
    InitialNode->q = q_init;
    InitialNode->control_input[0] = q_init[0];
    InitialNode->control_input[1] = q_init[1];
    InitialNode->cost2cum = 0; 
    InitialNode->parent_node = nullptr;
    InitialNode->start = true;

    std::vector<std::vector<double>> expansion_segment(2,std::vector<double>(2,0));

    // Store tree in unordered set
    std::unordered_set<Node*, NodePtrHasher, NodePtrComparator> tree;
    tree.insert(InitialNode);
    // Loop for sampling_trials number of times
    for(int i=0; i<sampling_trials; i++){

        std::vector<double> random_control_sample(2,0);
        sample_control_point(random_control_sample);
        
        // Find nearest neighbor using euclidean distance
        auto result_pair = find_nearest_neighbor(tree, random_control_sample);
        std::cout<<"Nearest neighbor found"<<result_pair.first->q[0]<<std::endl;
        expansion_segment[0] = (*(result_pair.first)).control_input;
        expansion_segment[1] = random_control_sample;

        // Connect sample to control point of nearest neighbor
        // Expand the state/node
        // For this test implementation, forget about obstacle and collision checking
        auto trajectory = segment_simulator((*result_pair.first).q, expansion_segment, result_pair.second);
        if(trajectory.size() == 0) continue;
        Node* new_node = new Node;
        new_node->q = trajectory[trajectory.size()-1];
        new_node->control_input = random_control_sample;
        new_node->parent_node = result_pair.first;
        new_node->is_forward = result_pair.second;

        tree.insert(new_node);
        std::cout<<"Number of nodes in the tree: "<<tree.size()<<std::endl;
    }
    std::cout<<"Tree size: "<<tree.size()<<std::endl;

    // Pick a random node from the tree and backtrack till first node
    Node* random_node = pick_random_node_from_tree(tree);
    Node* parent_node = random_node;

    std::vector<std::vector<double>> piecewise_path;

    while(!(*parent_node).start){
        piecewise_path.push_back((*random_node).control_input);
        parent_node = (*parent_node).parent_node;
    }

    piecewise_path.push_back((*parent_node).control_input);

    std::reverse(piecewise_path.begin(),piecewise_path.end());

    auto final_trajectory = forward_simulator(q_init, piecewise_path);

    // Save the trajectory into a file
	std::ofstream planned_trajectory;
	planned_trajectory.open("../output/PlannedTrajectory.txt", std::ios::trunc); // Creates new or replaces existing file
	if (!planned_trajectory.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	planned_trajectory << "This is a test trajectory computed for unit testing" << std::endl; // Description

    // Write tractor-trailer physical parameters into the file	planned_trajectory << "Forward lookahead radius: " << std::endl;
    planned_trajectory << FORWARD_LOOKAHEAD_DISTANCE << std::endl;
    planned_trajectory << "Backward lookahead radius: " << BACKWARD_LOOKAHEAD_DISTANCE << std::endl;
    planned_trajectory << "Tractor Wheelbase: " << TRACTOR_WHEELBASE << std::endl;
    planned_trajectory << "Trailer Wheelbase: " << TRAILER_WHEELBASE << std::endl;
    planned_trajectory << "Tractor hitch offset: " << TRACTOR_HITCH_OFFSET << std::endl;
    planned_trajectory << "Velocity" << VELOCITY << std::endl;
    // Tractor and Trailer Parameters

    planned_trajectory << std::endl;

    planned_trajectory << "Piecewise Linear Path" << std::endl;

    for (auto& segment : piecewise_path){
        planned_trajectory << segment[0] << ", " << segment[1] << std::endl;
    }

    for (auto& state : final_trajectory){
        planned_trajectory << state[0] << " " << state[1] << " " << state[2] << " " <<
        (M_PI - state[3]) << " " << state[4] << " " << state[5] << " " << state[6] << std::endl;
    }

	// planned_trajectory << "End of Trajectory Sequence" << std::endl;

    return piecewise_path;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// All test functions below this line ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

static void test_get_direction_fucntion(){

    Node* node;
    std::vector<double> control_sample(2,0);

    control_sample[0] = 8;
    control_sample[1] = 8;


    Node node1;
    node1.q[0] = 5;
    node1.q[1] = 5;
    node1.q[2] = M_PI_4 + M_PI_2;
    node1.q[3] = M_PI;
    // For this case the get_direction_fucntion must return false for reversing
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Trailer X: " << node1.q[0] << "     Trailer Y: " << node1.q[1] << "     Trailer theta is: " << node1.q[2] << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "Direction of simulator to use: " << get_direction(&node1, control_sample) << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    node1.q[0] = 5;
    node1.q[1] = 5;
    node1.q[2] = M_PI_4;
    node1.q[3] = M_PI;
    // For this case the get_direction_fucntion must return true for forward motion
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Trailer X: " << node1.q[0] << "     Trailer Y: " << node1.q[1] << "     Trailer theta is: " << node1.q[2] << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "Direction of simulator to use: " << get_direction(&node1, control_sample) << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    node1.q[0] = 5;
    node1.q[1] = 5;
    node1.q[2] = M_PI;
    node1.q[3] = M_PI;
    // For this case the get_direction_fucntion must return false for reversing
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Trailer X: " << node1.q[0] << "     Trailer Y: " << node1.q[1] << "     Trailer theta is: " << node1.q[2] << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "Direction of simulator to use: " << get_direction(&node1, control_sample) << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

}

static void test_sample_control_point(){

    // Node* node;
    std::vector<double> control_sample(2,0);

    sample_control_point(control_sample);
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    sample_control_point(control_sample);
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    sample_control_point(control_sample);
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;

    sample_control_point(control_sample);
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "Sampled Point X: " << control_sample[0] << "        Sampled Point Y: " << control_sample[1] << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
}

static void test_find_nearest_neighbor(){

    // Make a fake tree for testing
    std::unordered_set<Node*, NodePtrHasher, NodePtrComparator> test_tree;
    std::vector<double> control_sample(2,0);
    control_sample[0] = 2;
    control_sample[1] = 5;

    Node node1;
    node1.q[0] = 2;
    node1.q[1] = 0;
    node1.q[2] = M_PI_2;
    node1.q[3] = M_PI;
    node1.control_input[0] = 0;
    node1.control_input[1] = 0;


    Node node2;
    node2.q[0] = 2;
    node2.q[1] = 4;
    node2.q[2] = M_PI_2;
    node2.q[3] = M_PI;
    node2.control_input[0] = 5;
    node2.control_input[1] = 5;

    test_tree.insert(&node1);
    test_tree.insert(&node2);

    auto result1 = find_nearest_neighbor(test_tree, control_sample);
    auto result2 = find_nearest_neighbor(test_tree, control_sample);

    std::cout << "Node1 test X: " << result1.first->q[0] << std::endl;
    std::cout << "Node1 test Y: " << result1.first->q[1] << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << "Node2 test X: " << result2.first->q[0] << std::endl;
    std::cout << "Node2 test Y: " << result2.first->q[1] << std::endl;










}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// Main ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

int main(){

    std::vector<double> q_init(6,0);
    q_init[0] = 0.5;
    q_init[0] = 0.5;
    q_init[0] = M_PI_2;
    q_init[0] = M_PI;
    get_tractor_axle_center(q_init);

    std::vector<double> q_goal(6,0);
    q_goal[0] = 10.0;
    q_goal[0] = 10.0;
    q_goal[0] = M_PI_2;
    q_goal[0] = M_PI;
    get_tractor_axle_center(q_goal);

    // test_get_direction_fucntion();
    // test_sample_control_point();
    // test_find_nearest_neighbor();
    planner(q_init, q_goal);
    return 0;

}