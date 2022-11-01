#include <vector>
#include <stdexcept>

// Forward simulation function that takes initial configuration and a piecewise linear path
// as an input


//Forward simuation parameters

// forward lookahead radius
double forward_lookahead_radius = 0.5;

// backward lookahead radius
double backward_lookahead_radius = 1.0;

// tractor wheelbase
double tractor_wheelbase = 0.3;

// trailer wheelbase
double trailer_wheelbase = 0.8;

// Trailer hitch offset
double tractor_m = 0.2;

static void find_intersection_point(
    std::vector<double>& q_current, 
    std::vector<std::vector<double>>& piecewise_linear, 
    int& id_start, 
    int& id_goal, 
    std::vector<double>& intersection_point
){

    
    

    // Create circle equation for forward circle


    // Create circle equation for backward circle

    //

}

std::vector<double> forward_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> piecewise_linear
){

    // Check if inputs are valid
    if (piecewise_linear.size()<1){
        throw std::runtime_error("No piecewise linear input has been reeived.");
    }

    int id_start = 0;
    int id_goal = 1;

    std::vector<double> q_current=q_init;

    // Find the intersection points of the lookahead circle and piecewise linear path
    std::vector<double> intersection_point(2,0);

    find_intersection_point(q_init, piecewise_linear, id_start, id_goal, intersection_point);


    // Use the flag to determine the right forward/backward lookahead

    // Calculate the control inputs

    // Compute beta_e -> alpha_e

    // Integrate for timestep

}



//