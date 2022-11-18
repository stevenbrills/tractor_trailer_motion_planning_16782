#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <ct/optcon/optcon.h>
#include "matplotlibcpp.h"
#include "planner.h"

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

// Tractor tracking velocity
double velocity = 0.2;

namespace plt = matplotlibcpp;

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

/*This function takes in the current state, the direction of travel, the indices of the points along the piecewise
linear line and updates the return parameter "intersection_point" with the x and y coordinates of the target point that
should be used for the pure-pursuit algorithm*/
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
        double a = pow((piecewise_linear[id_goal][0] - piecewise_linear[id_start][0]),2) + 
        pow((piecewise_linear[id_goal][1] - piecewise_linear[id_start][1]),2);

        double b = 2*(
            ((piecewise_linear[id_start][0] - q_current[0])*(piecewise_linear[id_goal][0] - piecewise_linear[id_start][0])) + 
            ((piecewise_linear[id_start][1] - q_current[1])*(piecewise_linear[id_goal][1] - piecewise_linear[id_start][1]))
        );

        double c = pow((piecewise_linear[id_start][0] - q_current[0]),2) + pow((piecewise_linear[id_start][1] - q_current[1]),2) - backward_lookahead_radius;

        double det = pow(b,2) - 4*a*c;

        if(det < 0){
            std::cout << "No real roots" << std::endl;
        }

        // std::cout << "Determinant value could be near zero and fail equality, check performance " << std::endl;
        // std::cout << "Determinant value: " << det << std::endl;


        if(det>=0){

            double t1 = ((-1*b) + sqrt(det))/(2*a);
            double t2 = ((-1*b) - sqrt(det))/(2*a);

            // std::cout << t1 << " " << t2 << std::endl;

            if (check_double_equal(t1, t2)){

                // std::cout << (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0] << std::endl;
                // std::cout << (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1] << std::endl;


                intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];
                return true; // Intersection detected

            }
            else{

                // t1 will always be greater than t2 and since the second point of the piecewise linear
                // line will be the direction of travel, using t1 should give the point to drive to

                double x_intersection1 = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                double y_intersection1 = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];

                double x_intersection2 = (piecewise_linear[id_goal][0]*t2) + (1-t2)*piecewise_linear[id_start][0];
                double y_intersection2 = (piecewise_linear[id_goal][1]*t2) + (1-t2)*piecewise_linear[id_start][1];

                intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];

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
        double a = pow((piecewise_linear[id_goal][0] - piecewise_linear[id_start][0]),2) + 
        pow((piecewise_linear[id_goal][1] - piecewise_linear[id_start][1]),2);

        double b = 2*(
            ((piecewise_linear[id_start][0] - q_current[4])*(piecewise_linear[id_goal][0] - piecewise_linear[id_start][0])) + 
            ((piecewise_linear[id_start][1] - q_current[5])*(piecewise_linear[id_goal][1] - piecewise_linear[id_start][1]))
        );

        double c = pow((piecewise_linear[id_start][0] - q_current[0]),2) + pow((piecewise_linear[id_start][1] - q_current[1]),2) - backward_lookahead_radius;

        double det = pow(b,2) - 4*a*c;

        if(det < 0){
            std::cout << "No real roots" << std::endl;
        }

        // std::cout << "Determinant value could be near zero and fail equality, check performance " << std::endl;
        // std::cout << "Determinant value: " << det << std::endl;


        if(det>=0){

            double t1 = ((-1*b) + sqrt(det))/(2*a);
            double t2 = ((-1*b) - sqrt(det))/(2*a);

            if (check_double_equal(t1, t2)){

                intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];
                return true; // Intersection detected

            }
            else{

                // t1 will always be greater than t2 and since the second point of the piecewise linear
                // line will be the direction of travel, using t1 should give the point to drive to

                double x_intersection1 = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                double y_intersection1 = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];

                double x_intersection2 = (piecewise_linear[id_goal][0]*t2) + (1-t2)*piecewise_linear[id_start][0];
                double y_intersection2 = (piecewise_linear[id_goal][1]*t2) + (1-t2)*piecewise_linear[id_start][1];

                intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];

                return true;  //intersection detected

            }

        }
        else{
            std::cout << "No intersections detected" << std::endl;
        }

    }


    return false;
}

double get_beta_desired(
    const std::vector<double>& q_current, 
    const std::vector<double>& intersection_point,
    const bool& is_forward 
){

    if(!is_forward){

        // std::cout << "Delta x: " << (intersection_point[0] - q_current[0]) << std::endl;
        // std::cout << "Delta y: " << (intersection_point[1] - q_current[1]) << std::endl;
        // std::cout << "Theta value: " << q_current[2] << std::endl;
        // std::cout << "X Cos theta: " << (intersection_point[0] - q_current[0])*(cos(-1*q_current[2])) << std::endl;
        // std::cout << "Y Sin theta: " << (intersection_point[1] - q_current[1])*(sin(-1*q_current[2])) << std::endl;
        // std::cout << "X Sin theta: " << (intersection_point[0] - q_current[0])*(sin(-1*q_current[2])) << std::endl;
        // std::cout << "Y Cos theta: : " << (intersection_point[1] - q_current[1])*(cos(-1*q_current[2])) << std::endl;



        double projected_distance_on_axle = (intersection_point[0] - q_current[0])*(cos(-1*q_current[2])) + 
        (intersection_point[1] - q_current[1])*(sin(-1*q_current[2]));

        // double projected_distance_on_axle = (intersection_point[0] - q_current[0])*(cos(q_current[2] + M_PI_2)) + 
        // (intersection_point[1] - q_current[1])*(sin(q_current[2] + M_PI_2));

        double projected_distance_on_axle_normal = (intersection_point[0] - q_current[0])*(sin(-1*q_current[2])) + 
        (intersection_point[1] - q_current[1])*(cos(-1*q_current[2]));

        double theta_e = atan2(projected_distance_on_axle, projected_distance_on_axle_normal);

        double turning_circle_radius = fabs(backward_lookahead_radius/(2*sin(theta_e)));

        double beta_1 = atan(turning_circle_radius/trailer_wheelbase);

        double beta_2 = acos(tractor_m/sqrt(pow(trailer_wheelbase,2)+pow(turning_circle_radius,2)));

        double beta_d = (theta_e/fabs(theta_e))*(beta_1+beta_2);

        std::cout << "Projected distance on axle: " << projected_distance_on_axle << std::endl;

        std::cout << "Projected distance on axle normal: " << projected_distance_on_axle_normal << std::endl;

        std::cout << "atan2 value: " << theta_e << std::endl;

        std::cout << "Turning radius value is: " << turning_circle_radius << std::endl;
        
        std::cout << "Beta 1 componenet: " << beta_1 << std::endl;

        std::cout << "Beta 2 componenet: " << beta_2 << std::endl;

        // Theta_e is the heading error as decribed by the geometry of the pure-pursuit controller
        // with respect to the center-line of the trailer. When the trailer is facing upwards and
        // the centerline is extened downwards and the inersection point is below the rear axle,
        // counter clockwise angles from the centerline is positive and clockwise angles from the centerline
        // are negative.
        //              \
        //               \
        //                \
        //                 \
        //                  |
        //                  |
        //                  |
        //             ||+__|__-||
        //             ||   |   ||


        // double theta_e = asin(projected_distance_on_axle/backward_lookahead_radius);

        std::cout << "Calculated theta value is: " << theta_e << std::endl;

        // double beta_d = atan(((trailer_wheelbase*2*sin(theta_e))/backward_lookahead_radius));

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

// Function that returns beta_e
double get_beta_e_given_alpha(
    double& alpha_e
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double r2 = sqrt(pow(r1,2) + pow(tractor_m,2));

    double psi = atan(tractor_m/fabs(r1));

    double theta_1 = asin(fabs(r1)/r2);

    double theta_2 = acos(trailer_wheelbase/r2);

    // (alpha_e/fabs(alpha_e))*

    double beta_e = (alpha_e/fabs(alpha_e))*(theta_1 + theta_2);
    // ((M_PI_2 - psi) + acos(trailer_wheelbase/r2));

    return beta_e;

}

double get_alpha_e(
    double& beta_e
){

    // Find R3

    // double r3 = (trailer_wheelbase*sin(fabs(beta_e))/(cos(M_PI - fabs(beta_e)))) + tractor_m*sin(fabs(beta_e));

    double r3 = (tractor_m/cos(fabs(beta_e) - M_PI_2))*(((trailer_wheelbase*cos(M_PI - fabs(beta_e)))/tractor_m) + 1);

    double r2 = sqrt(pow(trailer_wheelbase,2) + pow(r3,2));

    double r1 = sqrt(pow(r2,2) - pow(tractor_m,2));

    double alpha_e = atan(tractor_wheelbase/r1)*(beta_e/fabs(beta_e));

    // std::cout << "R3 value: " << r3 << std::endl;
    // std::cout << "R2 value: " << r2 << std::endl;
    // std::cout << "R1 value: " << r1 << std::endl;


    // calculate R1
    // double r1 = sqrt(pow(r3,2) + pow(trailer_wheelbase,2) - pow(tractor_m,2));

    // calculate alpha_e
    // double alpha_e = atan((tractor_wheelbase*(beta_e/fabs(beta_e)))/r1);

    // std::cout << "Alpha_e value is: " << alpha_e << std::endl;

    return alpha_e;
}

// double get_gain(
//     double& alpha_e){

// }

std::vector<double> q_dot(
    const std::vector<double>& q_current,
    const double& alpha,
    const double& velocity
){
    std::vector<double> q_dot(4,0);

    double r1 = tractor_wheelbase/tan(alpha);

    double psi = -1*(alpha/fabs(alpha))*atan(tractor_m/fabs(r1));

    double va = (velocity/fabs(velocity))*fabs(((velocity*tractor_m*tan(alpha))/(tractor_wheelbase*sin(psi))));

    double vb = va*fabs(cos(psi-q_current[3]));

    double r2 = tractor_m/sin(psi);

    double r3 = trailer_wheelbase/sin(psi-q_current[3]);

    double omega_2 = ((r2)/(r1*-1*r3));

    q_dot[0] = vb*cos(q_current[2]);
    q_dot[1] = vb*sin(q_current[2]);
    q_dot[2] = velocity*((r2)/(r1*r3));
    q_dot[3] = velocity*(((r2)/(r1*r3)) - (1/r1));

    return q_dot;
}

double wrap_angle(
    const double& input_angle
){

    if((input_angle >= (-1*M_PI)) && (input_angle <= (M_PI))){
        return input_angle;
    }

    double remainder = fmod(input_angle,M_PI);

    if (remainder < 0){
        return (M_PI + remainder);
    }
    
    return ((-1*M_PI) + remainder);
}

std::vector<double> rk4_integrator(
const double& alpha,
const double& velocity,
const std::vector<double> q_current,
const double& timestep
){

    std::vector<double> k1(4,0);
    std::vector<double> k2(4,0);
    std::vector<double> k3(4,0);
    std::vector<double> k4(4,0);

    // Runge-Kutta Fourth Order Integration
    k1 = q_dot(q_current, alpha, velocity);
    std::vector<double> q_delta1(4,0);
    q_delta1[0] = q_current[0] + k1[0]*(timestep/2);
    q_delta1[1] = q_current[1] + k1[1]*(timestep/2);
    q_delta1[2] = q_current[2] + k1[2]*(timestep/2);
    q_delta1[3] = q_current[3] + k1[3]*(timestep/2);

    k2 = q_dot(q_delta1, alpha, velocity);
    std::vector<double> q_delta2(4,0);
    q_delta2[0] = q_current[0] + k2[0]*(timestep/2);
    q_delta2[1] = q_current[1] + k2[1]*(timestep/2);
    q_delta2[2] = q_current[2] + k2[2]*(timestep/2);
    q_delta2[3] = q_current[3] + k2[3]*(timestep/2);

    k3 = q_dot(q_delta2, alpha, velocity);
    std::vector<double> q_delta3(4,0);
    q_delta3[0] = q_current[0] + k3[0]*(timestep);
    q_delta3[1] = q_current[1] + k3[1]*(timestep);
    q_delta3[2] = q_current[2] + k3[2]*(timestep);
    q_delta3[3] = q_current[3] + k3[3]*(timestep);


    k4 = q_dot(q_delta3, alpha, velocity);

    std::vector<double> q_next(4,0);

    q_next[0] = q_current[0] + ((k1[0]/6)+(k2[0]/3)+(k3[0]/3)+(k4[0]/6))*timestep;
    q_next[1] = q_current[1] + ((k1[1]/6)+(k2[1]/3)+(k3[1]/3)+(k4[1]/6))*timestep;
    q_next[2] = q_current[2] + ((k1[2]/6)+(k2[2]/3)+(k3[2]/3)+(k4[2]/6))*timestep;
    q_next[3] = q_current[3] + ((k1[3]/6)+(k2[3]/3)+(k3[3]/3)+(k4[3]/6))*timestep;

    q_next[2] = wrap_angle(q_next[2]);
    q_next[3] = wrap_angle(q_next[3]);

    return q_next;

}



std::vector<double> forward_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> piecewise_linear
){

    // Constants
    float beta_prop_gain = 0.3;

    // Simulation time step
    double timestep = 0.001;

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

    // Use alpha_e to compute steering angle input
    // double alpha = alpha_e - get_gain(alpha_e)*(q_current[3] - beta_e);

    // Integrate motion through 






    return q_current;


    // Use the flag to determine the right forward/backward lookahead

    // Calculate the control inputs

    // Compute beta_e -> alpha_e

    // Integrate for timestep

}

////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// All test functions below this line ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

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
    // TODO: Update all test cases for the new calculation approach

    // Unit testing for find_intersection function
    std::vector<double> q_current{0,0,M_PI_2/2 + M_PI_2,0,0,0};
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
    double beta_e = 2.80456;
    std::cout << "Alpha e is: " << get_alpha_e(beta_e) << std::endl; // Expected alpha_e = 0.1

    beta_e = -1*2.80456;
    std::cout << "Alpha e is: " << get_alpha_e(beta_e) << std::endl; // Expected alpha_e = -0.1

    beta_e = 2.44194;
    std::cout << "Alpha e is: " << get_alpha_e(beta_e) << std::endl; // Expected alpha_e = 0.2

    beta_e = -1*2.44194;
    std::cout << "Alpha e is: " << get_alpha_e(beta_e) << std::endl; // Expected alpha_e = -0.2

    // Unit testing for find_intersection function
}

/*Test function with test cases and expected outputs for q_dot function*/
static void test_q_dot(){

    double velocity = 1;
    double alpha = 0.2;
    double beta_e;

    std::vector<double> q_current(4,0);
    std::vector<double> q_dot_vec(4,0);

    // First test case

    q_current[0] = 0.0;
    q_current[1] = 0.0;
    q_current[2] = M_PI_2;
    q_current[3] = 2.3;

    q_dot_vec = q_dot(q_current, alpha, velocity);
    beta_e = get_beta_e_given_alpha(alpha);

    // Expected values of rates are: xdot = ~0, ydot = 0.767, thetadot = 0.8196, betadot = 0.144
    std::cout << "--------------------------------" << std::endl;
    std::cout << "States are: " << std::endl;
    std::cout << " X: " << q_current[0] << " Y: " << q_current[1] << " Theta: " << q_current[2] << " Beta: " << q_current[3] << std::endl;
    std::cout << "Inputs are: " << std::endl;
    std::cout << "V: " << velocity << " Alpha: " << alpha << std::endl;
    std::cout << "Beta_e for given alpha is: " << beta_e << std::endl;
    std::cout << "Rates are: " << std::endl;
    std::cout << "X_dot: " << q_dot_vec[0] << " Y_dot: " << q_dot_vec[1] << " Theta_dot: " << q_dot_vec[2] << " Beta_dot: " << q_dot_vec[3] << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Second test case
    q_current[0] = 0.0;
    q_current[1] = 0.0;
    q_current[2] = M_PI_2;
    q_current[3] = 2.3;
    alpha = -0.2;
    beta_e = get_beta_e_given_alpha(alpha);

    q_dot_vec = q_dot(q_current, alpha, velocity);

    // Expected values of rates are: xdot = ~0, ydot = 0.5655, thetadot = 1.0447, betadot = 1.7204
    std::cout << "--------------------------------" << std::endl;
    std::cout << "States are: " << std::endl;
    std::cout << "X: " << q_current[0] << "Y: " << q_current[1] << "Theta: " << q_current[2] << "Beta: " << q_current[3] << std::endl;
    std::cout << "Inputs are: " << std::endl;
    std::cout << "V: " << velocity << "Alpha: " << alpha << std::endl;
    std::cout << "Beta_e for given alpha is: " << beta_e << std::endl;
    std::cout << "Rates are: " << std::endl;
    std::cout << "X_dot: " << q_dot_vec[0] << " Y_dot: " << q_dot_vec[1] << " Theta_dot: " << q_dot_vec[2] << " Beta_dot: " << q_dot_vec[3] << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Third test case
    q_current[0] = 0.0;
    q_current[1] = 0.0;
    q_current[2] = M_PI_2;
    q_current[3] = -2.3;
    alpha = 0.2;
    beta_e = get_beta_e_given_alpha(alpha);

    q_dot_vec = q_dot(q_current, alpha, velocity);

    // Expected values of rates are: xdot = ~0, ydot = 0.5655, thetadot = -1.0447, betadot = -1.7204
    std::cout << "--------------------------------" << std::endl;
    std::cout << "States are: " << std::endl;
    std::cout << "X: " << q_current[0] << "Y: " << q_current[1] << "Theta: " << q_current[2] << "Beta: " << q_current[3] << std::endl;
    std::cout << "Inputs are: " << std::endl;
    std::cout << "V: " << velocity << "Alpha: " << alpha << std::endl;
    std::cout << "Beta_e for given alpha is: " << beta_e << std::endl;
    std::cout << "Rates are: " << std::endl;
    std::cout << "X_dot: " << q_dot_vec[0] << " Y_dot: " << q_dot_vec[1] << " Theta_dot: " << q_dot_vec[2] << " Beta_dot: " << q_dot_vec[3] << std::endl;
    std::cout << "--------------------------------" << std::endl;

    // Fourth test case
    q_current[0] = 0.0;
    q_current[1] = 0.0;
    q_current[2] = M_PI_2;
    q_current[3] = -2.3;
    alpha = -0.2;
    beta_e = get_beta_e_given_alpha(alpha);

    q_dot_vec = q_dot(q_current, alpha, velocity);

    // Expected values of rates are: xdot = ~0, ydot = 0.767, thetadot = -0.8196, betadot = -0.144
    std::cout << "--------------------------------" << std::endl;
    std::cout << "States are: " << std::endl;
    std::cout << "X: " << q_current[0] << "Y: " << q_current[1] << "Theta: " << q_current[2] << "Beta: " << q_current[3] << std::endl;
    std::cout << "Inputs are: " << std::endl;
    std::cout << "V: " << velocity << "Alpha: " << alpha << std::endl;
    std::cout << "Beta_e for given alpha is: " << beta_e << std::endl;
    std::cout << "Rates are: " << std::endl;
    std::cout << "X_dot: " << q_dot_vec[0] << " Y_dot: " << q_dot_vec[1] << " Theta_dot: " << q_dot_vec[2] << " Beta_dot: " << q_dot_vec[3] << std::endl;
    std::cout << "--------------------------------" << std::endl;

}

static void test_rk4_integration_function(
    
){
    double velocity = 1;
    double alpha = 0.2;
    double timestep = 0.001;
    double beta_e;

    std::vector<double> q_current(4,0);
    std::vector<double> q_dot_vec(4,0);

    std::vector<std::vector<double>> state_history;



    // First test case
    double start_angle = M_PI_2;
    q_current[0] = 0.0;
    q_current[1] = 0.0;
    q_current[2] = wrap_angle(start_angle);
    q_current[3] = 2.3;

    state_history.push_back(q_current);

    double starttime = 0;
    double finaltime = 100;
    double steps = (finaltime - starttime)/timestep;

    std::vector<double> x(steps+1);
    std::vector<double> y(steps+1);
    std::vector<double> thetas(steps+1);
    std::vector<double> betas(steps+1);
    std::vector<double> time(steps+1);

    x[0] = q_current[0];
    y[0] = q_current[1];
    thetas[0] = q_current[2];
    betas[0] = q_current[3];
    time[0] = 0;

    for (int i=1; i<steps+1; i++){

        if(i==steps/2){
            alpha = -0.1;
        }
        q_current = rk4_integrator(alpha, velocity, q_current, timestep);
        x[i] = q_current[0];
        y[i] = q_current[1];
        thetas[i] = q_current[2];
        betas[i] = q_current[3];
        time[i] = i;
        state_history.push_back(q_current);
    }

    // auto figure = 
    // plt::figure(1);
    // plt::suptitle("Evolution");
    // plt::subplot(1,2,1);
    // plt::plot(time, x, "k-");
    // plt::subplot(1,2,2);
    // plt::plot(time, y, "r-");
    // plt::subplot(1,2,3);
    plt::plot(time, thetas, "b-");
    // plt::subplot(1,2,4);
    // plt::plot(time, betas, "g-");
    plt::show();

}

static void test_wrap_angle(){
// Test function to evaluate functionality of the wrap to -pi/+pi range

    double test_angle;

    test_angle = M_PI;

    std::cout << "PI after wrapping: " << wrap_angle(test_angle) << std::endl; // Expected value 3.14159

    test_angle = -1*M_PI;

    std::cout << "-PI after working: " << wrap_angle(test_angle) << std::endl; // Expected value -3.14159

    test_angle = 1.5*M_PI;

    std::cout << "1.5*PI after working: " << wrap_angle(test_angle) << std::endl; // Expected value -1.57

    test_angle = -1.5*M_PI;

    std::cout << "-1.5*PI after working: " << wrap_angle(test_angle) << std::endl; // Expected value 1.57

}

static void test_get_gain(){
    // Function to evaluate the correctness of the get_gain function which is defined in controller.cpp

    double alpha_e = 0.1;
    double beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e) << std::endl;

    alpha_e = -0.1;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e) << std::endl;


    alpha_e = 0.2;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e) << std::endl;

    alpha_e = -0.2;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e) << std::endl;

}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Main //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


int main(){

    // test_find_intersection_point();

    // test_get_beta_desired();

    // test_get_alpha_e();

    test_get_gain();

    gain_scheduler();

    // test_q_dot();

    // test_rk4_integration_function();

    // test_wrap_angle();

    return 0;

}

//