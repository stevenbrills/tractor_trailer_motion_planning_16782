#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

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

bool check_double_equal(
    double& a,
    double& b
){
    double epsilon = 0.001;

    if (abs(a-b)<=epsilon){
        return true;
    }

    return false;

}

static bool find_intersection_point(
    const std::vector<double>& q_current, 
    const std::vector<std::vector<double>>& piecewise_linear, 
    const int& id_start, 
    const int& id_goal,
    const bool& is_forward,
    std::vector<double>& intersection_point
){

    // if the motion is reverse, use the trailer centerd lookahead circle
    if (!is_forward){

        // Set up co-efficients for quadratic equation
        double a = pow((piecewise_linear[1][0] - piecewise_linear[0][0]),2) + 
        pow((piecewise_linear[1][1] - piecewise_linear[0][1]),2);

        double b = 2*(
            ((piecewise_linear[0][0] - q_current[0])*(piecewise_linear[1][0] - piecewise_linear[0][0])) + 
            ((piecewise_linear[0][1] - q_current[1])*(piecewise_linear[1][1] - piecewise_linear[0][1]))
        );

        double c = pow((piecewise_linear[0][0] - q_current[0]),2) + pow((piecewise_linear[0][1] - q_current[1]),2) - backward_lookahead_radius;

        double det = pow(b,2) - 4*a*c;

        if(det < 0){
            std::cout << "No real roots" << std::endl;
        }

        std::cout << "Determinant value could be near zero and fail equality, check performance " << std::endl;
        std::cout << "Determinant value: " << det << std::endl;


        if(det>=0){

            double t1 = ((-1*b) + sqrt(det))/(2*a);
            double t2 = ((-1*b) - sqrt(det))/(2*a);

            if (check_double_equal(t1, t2)){

                intersection_point[0] = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                intersection_point[1] = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];
                return true; // Intersection detected

            }
            else{

                // t1 will always be greater than t2 and since the second point of the piecewise linear
                // line will be the direction of travel, using t1 should give the point to drive to

                double x_intersection1 = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                double y_intersection1 = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];

                double x_intersection2 = (piecewise_linear[1][0]*t2) + (1-t2)*piecewise_linear[0][0];
                double y_intersection2 = (piecewise_linear[1][1]*t2) + (1-t2)*piecewise_linear[0][1];

                intersection_point[0] = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                intersection_point[1] = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];

                return true;  //intersection detected

            }

        }
        else{
            std::cout << "No intersections detected" << std::endl;
        }

    }

    // If the motion is forward, use tractor lookahead circle
    else{

        // Set up co-efficients for quadratic equation
        double a = pow((piecewise_linear[1][0] - piecewise_linear[0][0]),2) + 
        pow((piecewise_linear[1][1] - piecewise_linear[0][1]),2);

        double b = 2*(
            ((piecewise_linear[0][0] - q_current[4])*(piecewise_linear[1][0] - piecewise_linear[0][0])) + 
            ((piecewise_linear[0][1] - q_current[5])*(piecewise_linear[1][1] - piecewise_linear[0][1]))
        );

        double c = pow((piecewise_linear[0][0] - q_current[0]),2) + pow((piecewise_linear[0][1] - q_current[1]),2) - backward_lookahead_radius;

        double det = pow(b,2) - 4*a*c;

        if(det < 0){
            std::cout << "No real roots" << std::endl;
        }

        std::cout << "Determinant value could be near zero and fail equality, check performance " << std::endl;
        std::cout << "Determinant value: " << det << std::endl;


        if(det>=0){

            double t1 = ((-1*b) + sqrt(det))/(2*a);
            double t2 = ((-1*b) - sqrt(det))/(2*a);

            if (check_double_equal(t1, t2)){

                intersection_point[0] = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                intersection_point[1] = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];
                return true; // Intersection detected

            }
            else{

                // t1 will always be greater than t2 and since the second point of the piecewise linear
                // line will be the direction of travel, using t1 should give the point to drive to

                double x_intersection1 = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                double y_intersection1 = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];

                double x_intersection2 = (piecewise_linear[1][0]*t2) + (1-t2)*piecewise_linear[0][0];
                double y_intersection2 = (piecewise_linear[1][1]*t2) + (1-t2)*piecewise_linear[0][1];

                intersection_point[0] = (piecewise_linear[1][0]*t1) + (1-t1)*piecewise_linear[0][0];
                intersection_point[1] = (piecewise_linear[1][1]*t1) + (1-t1)*piecewise_linear[0][1];

                return true;  //intersection detected

            }

        }
        else{
            std::cout << "No intersections detected" << std::endl;
        }

    }


    return false;


    
    

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