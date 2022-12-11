#ifndef TRACTOR_TRAILER_PLANNER_HEADER_GUARD
#define TRACTOR_TRAILER_PLANNER_HEADER_GUARD

#include <vector>
#include <string>
#include "cc.hpp"

#define TRACTOR_WHEELBASE 0.3
#define TRACTOR_HITCH_OFFSET 0.2
#define TRAILER_WHEELBASE 0.8
#define FORWARD_LOOKAHEAD_DISTANCE 1.0
#define BACKWARD_LOOKAHEAD_DISTANCE 2.0
#define VELOCITY 0.2
#define EPSILON 5
#define GOAL_XY_TOLERANCE 5

struct Node{
    std::vector<double> q;
    std::vector<double> control_input;
    Node* parent_node;
    double cost2cum;
    bool start;
    bool is_forward;

    Node(){
        this->q = {0,0,0,0,0,0};
        this->control_input = {0,0};
        this->cost2cum = 0;
        this->start = false;
        this->is_forward = false;
    }

    std::string toString() const
    {
        std::string temp = "";
        temp += std::to_string(this->q[0]) + "," + std::to_string(this->q[1]) + "," + std::to_string(this->q[2]) + "," + std::to_string(this->q[3]);
        return temp;
    }
};

struct NodePtrHasher{
    size_t operator()(const Node* node) const
    {
        return std::hash<std::string>{}(node->toString());
    }
};

struct NodePtrComparator{
    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return (*lhs).q == (*rhs).q;
    }
};

std::vector<std::vector<double>> segment_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> segment,
    const bool is_forward,
    CollisionCheck& cc,
    double* map,
    const int& x_size,
    const int& y_size
);

std::vector<std::vector<double>> segment_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> segment,
    bool is_forward
);

std::vector<std::vector<double>> forward_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> piecewise_linear
);


double get_gain(
    const double& beta_e,
    const double& alpha_e,
    const double& velocity
);

bool check_double_equal(
    double& a,
    double& b
);

void get_tractor_axle_center(
    std::vector<double>& q
);

void gain_scheduler();

double get_beta_e_given_alpha(
    double& alpha_e
);

#endif