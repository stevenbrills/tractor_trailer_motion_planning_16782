#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

// Forward simulation function that takes initial configuration and a piecewise linear path
// as an input


//Forward simuation parameters

// forward lookahead radius
double forward_lookahead_radius = 1.0;

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

    if (fabs(a-b)<=epsilon){
        return true;
    }

    return false;

}

double get_beta_desired(
    const std::vector<double>& q_current, 
    const std::vector<double>& intersection_point,
    const bool& is_forward 
){

    if(!is_forward){

        double projected_distance_on_axle = (intersection_point[0] - q_current[0])*(cos(q_current[2] + M_PI_2)) + 
        (intersection_point[1] - q_current[1])*(sin(q_current[2] + M_PI_2));

        double projected_distance_on_axle_normal = (intersection_point[0] - q_current[0])*(cos(q_current[2] + M_PI)) + 
        (intersection_point[1] - q_current[1])*(sin(q_current[2] + M_PI));

        double atan2val = atan2(projected_distance_on_axle_normal, projected_distance_on_axle);

        std::cout << "Projected distance on axle: " << projected_distance_on_axle << std::endl;

        std::cout << "Projected distance on axle normal: " << projected_distance_on_axle_normal << std::endl;

        std::cout << "atan2 value: " << atan2val << std::endl;

        double theta_e=0.0;

        // Theta_e is the heading error as decribed by the geometry of the pure-pursuit controller
        // with respect to the center-line of the trailer. When the trailer is facing upwards and
        // the centerline is extened downwards and the inersection point is below the rear axle,
        // counter clockwise angles from the centerline is negative and clockwise angles from the centerline
        // is positive.
        //                  |
        //                  |
        //                  |
        //             ||___|___||
        //             ||+  |  -||

        if(atan2val>=0 && atan2val<=M_PI_2){
            theta_e = M_PI_2 - atan2val;
        }
        else if(atan2val>M_PI_2 && atan2val<=M_PI_2){
            theta_e = M_PI_2 - atan2val;
        }
        else if(atan2val<0 && atan2val>=(-1*M_PI_2)){
            theta_e = M_PI_2 + (-1*atan2val);
        }
        else{
            theta_e = -1*(M_PI_2*3) + (-1*atan2val);            
        }

        // double theta_e = asin(projected_distance_on_axle/backward_lookahead_radius);

        std::cout << "Calculated theta value is: " << theta_e << std::endl;

        double beta_d = atan(((trailer_wheelbase*2*sin(theta_e))/backward_lookahead_radius));

        std::cout << "Beta desired is: " << beta_d << std::endl;

        return beta_d;
    }

    return 0.0;

    /*
    double projected_distance_on_axle = (intersection_point[0] - q_current[4])*(cos(q_current[2] + M_PI_2)) + 
    (intersection_point[1] - q_current[5])*(sin(q_current[2] + M_PI_2));

    double projected_distance_on_axle_normal = (intersection_point[0] - q_current[4])*(cos(q_current[2] + M_PI)) + 
    (intersection_point[1] - q_current[5])*(sin(q_current[2] + M_PI));

    double atan2val = atan2(projected_distance_on_axle_normal, projected_distance_on_axle);

    std::cout << "Projected distance on axle: " << projected_distance_on_axle << std::endl;

    std::cout << "Projected distance on axle normal: " << projected_distance_on_axle_normal << std::endl;

    std::cout << "atan2 value: " << atan2val << std::endl;

    double theta_e=0.0;

    // Theta_e is the heading error as decribed by the geometry of the pure-pursuit controller
    // with respect to the center-line of the trailer. When the trailer is facing upwards and
    // the centerline is extened downwards and the inersection point is below the rear axle,
    // counter clockwise angles from the centerline is negative and clockwise angles from the centerline
    // is positive.
    //                  |
    //                  |
    //                  |
    //             ||___|___||
    //             ||+  |  -||

    if(atan2val>=0 && atan2val<=M_PI_2){
        theta_e = M_PI_2 - atan2val;
    }
    else if(atan2val>M_PI_2 && atan2val<=M_PI_2){
        theta_e = M_PI_2 - atan2val;
    }
    else if(atan2val<0 && atan2val>=(-1*M_PI_2)){
        theta_e = M_PI_2 + (-1*atan2val);
    }
    else{
        theta_e = -1*(M_PI_2*3) + (-1*atan2val);            
    }

    // double theta_e = asin(projected_distance_on_axle/backward_lookahead_radius);

    std::cout << "Calculated theta value is: " << theta_e << std::endl;

    double beta_d = atan(((trailer_wheelbase*2*sin(theta_e))/backward_lookahead_radius));

    std::cout << "Beta desired is: " << beta_d << std::endl;

    return beta_d;

    */
}

double get_alpha_e(
    double& beta_e
){

    // Find R3
    double r3 = (trailer_wheelbase + (tractor_m/cos(beta_e)))/(tan(beta_e));

    // calculate R1
    double r1 = sqrt(pow(r3,2) + pow(trailer_wheelbase,2) - pow(tractor_m,2));

    // calculate alpha_e
    double alpha_e = atan((tractor_wheelbase*(beta_e/fabs(beta_e)))/r1);

    std::cout << "Alpha_e value is: " << alpha_e << std::endl;

    return alpha_e;
}

// A matrix
double get_state_matrix(
    double alpha_e,
    std::vector<double> q_current
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = atan(tractor_m/r1);

    double A = ((tan(alpha_e)/tractor_wheelbase) - 
    ((tractor_m/(tractor_wheelbase*trailer_wheelbase))*tan(alpha_e)*((-1*sin(q_current[3]))+(-1*(cos(q_current[3])/tan(psi))))));

    return A;
}

// B matrix
double get_input_matrix(
    double alpha_e,
    std::vector<double> q_current
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = atan(tractor_m/r1);

    double B = ((1/(cos(alpha_e)*cos(alpha_e)*tractor_wheelbase)) - 
    ((tractor_m/(tractor_wheelbase*trailer_wheelbase*cos(alpha_e)*cos(alpha_e)))*
    ((cos(q_current[3]))+(-1*(sin(q_current[3])/tan(psi))))));

    return B;
}

// 



std::vector<double> gain_scheduler(
){

    // State transition model for beta_dot

    // State Matrix
    double 

    // Constants
    float beta_prop_gain = 0.3;

    // Check if inputs are valid
    if (piecewise_linear.size()<1){
        throw std::runtime_error("No piecewise linear input has been reeived.");
    }

    bool is_forward = false;

    int id_start = 0;
    int id_goal = 1;

    std::vector<double> q_current=q_init;

    // Find the intersection points of the lookahead circle and piecewise linear path
    std::vector<double> intersection_point(2,0);

    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point);

    // beta desired works only for the reversing simulation now, need to create a different approach for the forward motion
    double beta_desired = get_beta_desired(q_current, intersection_point, is_forward);

    // Add the proportional gain beta
    double beta_e = beta_desired + beta_prop_gain*(beta_desired - q_current[3]);

    // Get the value of alpha_e from beta_e using the pre-compensation link
    double alpha_e = get_alpha_e(beta_desired);


    return q_current;


    // Use the flag to determine the right forward/backward lookahead

    // Calculate the control inputs

    // Compute beta_e -> alpha_e

    // Integrate for timestep

}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// All test functions below this line ////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// All test functions below this line ////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////

/*Test function with test cases and expected outputs for find_intersection_point_function*/
static void test_find_intersection_point(){

    // Unit testing for find_intersection function
    std::vector<double> q_current{0,0,1.57,0,0,0};
    std::vector<std::vector<double>> piecewise_linear(2, std::vector<double>(2,0));
    int id_start = 0;
    int id_goal = 1;
    bool is_forward =false;
    std::vector<double> intersection_point(2,0);

    piecewise_linear[0][0] = 1;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 1;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=1, y=0
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;


    piecewise_linear[0][0] = -1;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 1;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=0.707, y=0.707
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;

    piecewise_linear[0][0] = 1;
    piecewise_linear[0][1] = 1;
    piecewise_linear[1][0] = -1;
    piecewise_linear[1][1] = -1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=-0.707, y=-0.707
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;

    piecewise_linear[0][0] = 0.5;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 0.5;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=0.5, y=0.866
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;

    piecewise_linear[0][0] = 0.5;
    piecewise_linear[0][1] = 1;
    piecewise_linear[1][0] = 0.5;
    piecewise_linear[1][1] = -1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=0.5, y=-0.866
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;

}

/*Test function with test cases and expected outputs for get_beta_desired function*/
static void test_get_beta_desired(){

    // TODO: Add beta desired expected values as well

    // Unit testing for find_intersection function
    std::vector<double> q_current{0,0,M_PI_2,0,0,0};
    std::vector<std::vector<double>> piecewise_linear(2, std::vector<double>(2,0));
    int id_start = 0;
    int id_goal = 1;
    bool is_forward =false;
    std::vector<double> intersection_point(2,0);

    piecewise_linear[0][0] = 1;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 1;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: theta_e = -pi/2
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);

    piecewise_linear[0][0] = -1;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = -1;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: theta_e = pi/2
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);

    piecewise_linear[0][0] = 0.5;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 0.5;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: theta_e = -2.61799
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);

    piecewise_linear[0][0] = -0.5;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = -0.5;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: theta_e = 2.61799
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);

    piecewise_linear[0][0] = 0.5;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = 0.5;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: theta_e = 
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);

    piecewise_linear[0][0] = -0.5;
    piecewise_linear[0][1] = -1;
    piecewise_linear[1][0] = -0.5;
    piecewise_linear[1][1] = 1;
    find_intersection_point(q_current, piecewise_linear, id_start, id_goal, is_forward, intersection_point); // Expected: x=1, y=0
    std::cout << "Intersection x is: " << intersection_point[0] << ", Intersection y is: " << intersection_point[1] << std::endl;
    get_beta_desired(q_current, intersection_point, is_forward);
}

/*Test function with test cases and expected outputs for get_beta_desired function*/
static void test_get_alpha_e(){

    // Test function expected values are for trailer wheelbase of 0.8 m, tractor wheelbase of 0.3 m and
    // tractor offset of 0.2 m

    // TODO: Add alpha_e expected values
    double beta_e = +0.785398;  // for beta_e = 45 degrees, alhpa_e using online solver should be 0.221639362 radians
    get_alpha_e(beta_e);

    beta_e = -0.785398;
    get_alpha_e(beta_e); // for beta_e = -45 degrees, alhpa_e using online solver should be -0.221639362 radians

    beta_e = 0.436332;
    get_alpha_e(beta_e); // for beta_e = 25 degrees, alhpa_e using online solver should be 0.12849 radians

    beta_e = 1.39626;
    get_alpha_e(beta_e); // for beta_e = 80 degrees (1.39626 radians) using online solver alpha_e should be 0.340182125 radians


    // Unit testing for find_intersection function


}


int main(){

    // test_find_intersection_point();

    // test_get_beta_desired();

    test_get_alpha_e();

    return 0;

}

//