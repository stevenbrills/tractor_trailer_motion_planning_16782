#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <ct/optcon/optcon.h>

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

// A matrix for equillibrium point
double get_state_matrix(
    double& alpha_e,
    double& beta_e
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = atan(tractor_m/r1);

    double A = ((tan(alpha_e)/tractor_wheelbase) - 
    ((tractor_m/(tractor_wheelbase*trailer_wheelbase))*tan(alpha_e)*((-1*sin(beta_e))+(-1*(cos(beta_e)/tan(psi))))));

    return A;
}

// B matrix for equillibrium point
double get_input_matrix(
    double& alpha_e,
    double& beta_e
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = atan(tractor_m/r1);

    double B = ((1/(cos(alpha_e)*cos(alpha_e)*tractor_wheelbase)) - 
    ((tractor_m/(tractor_wheelbase*trailer_wheelbase*cos(alpha_e)*cos(alpha_e)))*
    ((cos(beta_e))+(-1*(sin(beta_e)/tan(psi))))));

    return B;
}

// Function that returns beta_e
double get_beta_e_given_alpha(
    double& alpha_e
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double r2 = sqrt(pow(r1,2) + pow(tractor_m,2));

    double r3 = sqrt(pow(r2,2) - pow(trailer_wheelbase,2));

    double psi = atan(tractor_m/r1);

    double beta_e = atan(trailer_wheelbase/r3) + psi;

    return beta_e;

}

// 


void gain_scheduler(
){

    const size_t state_dim = 1;
    const size_t control_dim = 1;

    // double R = 0.0;
    // double Q = 10.0;

    std::cout << "In the gain scheduler function" << std::endl;

    // Loop through values of alpha_e
    // Calculate 

    // Constants
    float beta_prop_gain = 0.3;

    // calculate the extents of alpha_e
    double alpha_e_max = atan(tractor_wheelbase/sqrt(pow(trailer_wheelbase,2) - pow(tractor_m,2)));

    // loop from -alpha_e_max to +alpha_e_max

    // for a given alpha_e, compute corresponding beta_e

    // Using beta_e and alpha_e, compute state and input matrices for the equillibrium point

    double alpha_e = 0.1;
    double beta_e = 0.1;

    ct::core::StateMatrix<state_dim> A;
    ct::core::ControlMatrix<control_dim> B;
    ct::core::StateMatrix<state_dim> Q;
    ct::core::ControlMatrix<control_dim> R;



    A(0,0) = get_state_matrix(alpha_e, beta_e);
    B(0,0) = get_input_matrix(alpha_e, beta_e);
    Q(0,0) = 10;
    R(0,0) = 10;


    double test = get_state_matrix(alpha_e, beta_e);

    std::cout << "Returned value of get_state_matrix: " << test << std::endl;

    // double B = get_input_matrix(alpha_e, beta_e);




    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;

    std::cout << "A: " << std::endl << A << std::endl << std::endl;
    std::cout << "B: " << std::endl << B << std::endl << std::endl;
    std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
    std::cout << "R: " << std::endl << R << std::endl << std::endl;

    lqrSolver.compute(Q, R, A, B, K);

    std::cout << "LQR gain matrix:" << std::endl << K << std::endl;





    // State Matrix
    // double A = get_state_matrix(alpha_e, q_current);

    // Input Matrix
    // double B = get_input_matrix(alpha_e, q_current);

    // const size_t state_dim = 1;
    // const size_t control_dim = 1;

    // design the LQR controller
    // ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    // ct::core::FeedbackMatrix<state_dim, control_dim> K;

    // double Q = 10;
    // double R = 0;

    // lqrSolver.compute(Q, R, A, B, K);

    bool is_forward = false;

    int id_start = 0;
    int id_goal = 1;


    // Use the flag to determine the right forward/backward lookahead

    // Calculate the control inputs

    // Compute beta_e -> alpha_e

    // Integrate for timestep

}


int main(){

    // test_find_intersection_point();

    // test_get_beta_desired();

    // test_get_alpha_e();

    gain_scheduler();

    return 0;

}

//