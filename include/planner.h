#ifndef TRACTOR_TRAILER_PLANNER_HEADER_GUARD
#define TRACTOR_TRAILER_PLANNER_HEADER_GUARD

struct Node{
    std::vector<double> q;
    std::vector<double> control_input;
    Node* parent_node;
    double cost2cum;

    Node(){
        this->q = {0,0,0,0};
        this->control_input = {0,0};
        this->cost2cum = 0;

    }

    string toString() const
    {
        std::string temp = "";
        temp += std::to_string(this->q[0]) + "," + std::to_string(this->q[1]) + "," + std::to_string(this->q[2]) + "," + std::to_string(this->q[3]);
        return temp;
    }
};

struct NodeHasher{
    size_t operator()(const Node* node) const
    {
        return std::hash<std::string>{}(node->toString());
    }
}

struct NodeComparator{

    bool operator()(const Node* lhs, const Node* rhs) const
    {
        return (*lhs)->q == (*rhs)->q;
    }

}


double get_gain(
    const double& beta_e,
    const double& alpha_e,
    const double& velocity
);

void gain_scheduler();

double get_beta_e_given_alpha(
    double& alpha_e
);

#endif