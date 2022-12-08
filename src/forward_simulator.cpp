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
double forward_lookahead_radius = 1;

// backward lookahead radius
double backward_lookahead_radius = 2;

// tractor wheelbase
double tractor_wheelbase = 0.3;

// trailer wheelbase
double trailer_wheelbase = 0.8;

// Trailer hitch offset
double tractor_m = 0.2;

// Tractor tracking velocity
double velocity = 0.2;

// Alpha E Max
double alpha_e_max = atan(tractor_wheelbase/sqrt(pow(trailer_wheelbase,2) - pow(tractor_m,2)));

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

        double c = pow((piecewise_linear[id_start][0] - q_current[0]),2) + pow((piecewise_linear[id_start][1] - q_current[1]),2) - pow(backward_lookahead_radius,2);

        double det = pow(b,2) - (4*a*c);

        // std::cout << "A: " << a << " B: " << b << " C: " << c << std::endl;

        // if(det < 0){
        //     std::cout << "No real roots" << std::endl;
        //     return false;
        // }

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
                // std::cout << "equal intersection points" << std::endl;
                // std::cout << "id_start: " << id_start << "id_goal: " << id_goal << std::endl;
                if((t1>=0) && (t1 <=1)){
                    return true; 
                } //intersection detected
                else{
                    // std::cout << "t1 is out of bounds reverse equal" << std::endl;
                    return false;
                }

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

                // std::cout << "UNEQUAL intersection points" << std::endl;

                // std::cout << "id_start: " << id_start << "id_goal: " << id_goal << std::endl;

                if((t1>=0) && (t1 <=1)){
                    return true; 
                } //intersection detected
                else{
                    // std::cout << "t1 is out of bounds reverse unequal" << std::endl;
                    return false;
                }

            }

        }
        // else{
        // std::cout << "No real roots: " << "id_start: " << id_start << "id_goal: " << id_goal << std::endl;
        std::cout << "No real roots: " << std::endl;
        return false;

        // }

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

        double c = pow((piecewise_linear[id_start][0] - q_current[4]),2) + pow((piecewise_linear[id_start][1] - q_current[5]),2) - pow(forward_lookahead_radius,2);

        double det = pow(b,2) - 4*a*c;

        // std::cout << "Determinant value could be near zero and fail equality, check performance " << std::endl;
        // std::cout << "Determinant value: " << det << std::endl;


        if(det>=0){

            double t1 = ((-1*b) + sqrt(det))/(2*a);
            double t2 = ((-1*b) - sqrt(det))/(2*a);

            if (check_double_equal(t1, t2)){

                if((t1>=0) && (t1 <=1)){
                intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];
                return true; // Intersection detected
                }
                else{
                    // std::cout << "t1 is out of bounds: " << t1 << std::endl;
                    // std::cout << "t2 is: " << t2 << std::endl;
                    return false;
                }



            }
            else{

                // t1 will always be greater than t2 and since the second point of the piecewise linear
                // line will be the direction of travel, using t1 should give the point to drive to

                double x_intersection1 = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                double y_intersection1 = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];

                double x_intersection2 = (piecewise_linear[id_goal][0]*t2) + (1-t2)*piecewise_linear[id_start][0];
                double y_intersection2 = (piecewise_linear[id_goal][1]*t2) + (1-t2)*piecewise_linear[id_start][1];
                if((t1>=0) && (t1 <=1)){
                    intersection_point[0] = (piecewise_linear[id_goal][0]*t1) + (1-t1)*piecewise_linear[id_start][0];
                    intersection_point[1] = (piecewise_linear[id_goal][1]*t1) + (1-t1)*piecewise_linear[id_start][1];
                    return true;
                }
                else{
                    // std::cout << "t1 is out of bounds: " << t1 << std::endl;
                    // std::cout << "t2 is: " << t2 << std::endl;
                    return false;
                }
  //intersection detected

            }

        }

        std::cout << "No real roots" << std::endl;
        return false;

    }


    return false;
}

double get_beta_desired(
    const std::vector<double>& q_current, 
    const std::vector<double>& intersection_point,
    const bool& is_forward 
){



    // Transform the intersection point into the local frame of the vehicle
    double tx = -1*((q_current[0]*cos(q_current[2])) + (q_current[1]*sin(q_current[2])));
    double ty = -1*((-1*q_current[0]*sin(q_current[2])) + (q_current[1]*cos(q_current[2])));



    // double projected_distance_on_axle = (intersection_point[0] - q_current[0])*(sin(-1*q_current[2])) + 
    // (intersection_point[1] - q_current[1])*(cos(-1*q_current[2]));

    double projected_distance_on_axle_normal = (intersection_point[0]*cos(q_current[2])) + (intersection_point[1]*sin(q_current[2])) + tx;

    double projected_distance_on_axle = (-1*intersection_point[0]*sin(q_current[2])) + (intersection_point[1]*cos(q_current[2])) + ty;


    // double projected_distance_on_axle = (intersection_point[0] - q_current[0])*(cos(q_current[2] + M_PI_2)) + 
    // (intersection_point[1] - q_current[1])*(sin(q_current[2] + M_PI_2));

    // double projected_distance_on_axle_normal = (intersection_point[0] - q_current[0])*(cos(-1*q_current[2])) + 
    // (intersection_point[1] - q_current[1])*(sin(-1*q_current[2]));

    double theta_e = atan2(projected_distance_on_axle, projected_distance_on_axle_normal);

    double turning_circle_radius = fabs(backward_lookahead_radius/(2*sin(theta_e)));

    double beta_1 = atan(turning_circle_radius/trailer_wheelbase);

    double beta_2 = acos(tractor_m/sqrt(pow(trailer_wheelbase,2)+pow(turning_circle_radius,2)));

    double beta_d = (theta_e/fabs(theta_e))*(beta_1+beta_2);

    // std::cout << "Projected distance on axle: " << projected_distance_on_axle << std::endl;

    // std::cout << "Projected distance on axle normal: " << projected_distance_on_axle_normal << std::endl;

    // std::cout << "atan2 value: " << theta_e << std::endl;

    // std::cout << "Turning radius value is: " << turning_circle_radius << std::endl;
    
    // std::cout << "Beta 1 componenet: " << beta_1 << std::endl;

    // std::cout << "Beta 2 componenet: " << beta_2 << std::endl;

    // Theta_e is the heading error as decribed by the geometry of the pure-pursuit controller
    // with respect to the center-line of the trailer. When the trailer is facing upwards and
    // the centerline extends upwards shown below counter clockwise angles from the centerline
    // are positive and clockwise angles from the centerline are negative.
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

    // std::cout << "Calculated theta value is: " << theta_e << std::endl;

    // double beta_d = atan(((trailer_wheelbase*2*sin(theta_e))/backward_lookahead_radius));

    // std::cout << "Beta desired is: " << beta_d << std::endl;

    return beta_d;

    

    // return 0.0;

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

double get_tractor_orientation(
    const std::vector<double>& q_current
){

    if(q_current[3]>=0){
        return q_current[2] + (M_PI - q_current[3]);
    }

    return (q_current[2] - (M_PI - fabs(q_current[3])));    
}

void get_tractor_axle_center(
    std::vector<double>& q
){

    double tractor_theta = get_tractor_orientation(q);
    // std::cout << "X of trailer: " << q[0] << "  Y of trailer: " << q[1] << std::endl;
    // std::cout << "Cos of trailer angle: " << cos(q[3]) << std::endl;
    // std::cout << "Cos of tractor angle: " << cos(tractor_theta) << std::endl;

    q[4] = q[0] + trailer_wheelbase*cos(q[2]) + tractor_m*cos(tractor_theta);
    q[5] = q[1] + trailer_wheelbase*sin(q[2]) + tractor_m*sin(tractor_theta);

}

double wrap_angle(
    const double input_angle
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

double clip_to_alpha_e(
    double angle
){
    if(angle<(-1*alpha_e_max)){
        return (-1*alpha_e_max);
    }

    if(angle>(1*alpha_e_max)){
        return (1*alpha_e_max);
    }

    return angle;

}


double get_alpha_for_forward_motion(
    const std::vector<double>& intersection_point,
    const std::vector<double>& q_current
){

    // Frame to be transformed into is the tractor axle frame
    // Use the axle center x,y and theta of tractor
    double theta_tractor = get_tractor_orientation(q_current);
    // std::cout << "Orientation of the tractor is: " << theta_tractor << std::endl;
    // std::cout << "X of tractor: " << q_current[4] << "Y of tractor: " << q_current[5] << std::endl;

    // Transform the intersection point into the local frame of the vehicle
    double tx = -1*((q_current[4]*cos(theta_tractor)) + (q_current[5]*sin(theta_tractor)));
    double ty = -1*((-1*q_current[4]*sin(theta_tractor)) + (q_current[5]*cos(theta_tractor)));

    // Projected distances 
    double projected_distance_on_axle_normal = (intersection_point[0]*cos(theta_tractor)) + (intersection_point[1]*sin(theta_tractor)) + tx;
    double projected_distance_on_axle = (-1*intersection_point[0]*sin(theta_tractor)) + (intersection_point[1]*cos(theta_tractor)) + ty;

    double r1 = (pow(forward_lookahead_radius,2))/(2*fabs(projected_distance_on_axle));

    double alpha=0;

    if(projected_distance_on_axle==0){
        alpha = 0;
    }
    else{
        alpha = atan(tractor_wheelbase/r1)*(projected_distance_on_axle/fabs(projected_distance_on_axle));
    }

    alpha = wrap_angle(alpha);



    // std::cout << "Computed alpha value is: " << alpha << std::endl;
    if(std::isnan(alpha)){
        std::cout << "Tractor theta: " << theta_tractor << std::endl;
        std::cout << "tx: " << tx << std::endl;
        std::cout << "ty: " << ty << std::endl;
        std::cout << "projected_distance_on_axle_normal: " << projected_distance_on_axle_normal << std::endl;
        std::cout << "projected_distance_on_axle: " << projected_distance_on_axle << std::endl;
        std::cout << "R1: " << r1 << std::endl;
    }

    return alpha;
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

    double va=0;
    double r1=0;
    double psi=0;
    double r2=0;
    double r3=0;
    double vb=0;
    double omega_2=0;
    double trailer_theta_dot = 0;
    double hitch_turn_radius_zero_alpha = 0;
    double phi = 0;
    double beta_dot = 0;
    double omega_1 = 0;
    
    r1 = tractor_wheelbase/tan(alpha);

    psi = -1*(alpha/fabs(alpha))*atan(tractor_m/fabs(r1));

    va = (velocity/fabs(velocity))*fabs(((velocity*tractor_m*tan(alpha))/(tractor_wheelbase*sin(psi))));

    vb = va*fabs(cos(psi-q_current[3]));

    r2 = tractor_m/sin(psi);

    r3 = trailer_wheelbase/sin(psi-q_current[3]);

    omega_1 = velocity*(1/r1);

    omega_2 = velocity*((r2)/(r1*r3));

    // if(r3<0){
    //     omega_2 = ((r2)/(r1*-1*r3));
    // }
    // else{
    //     omega_2 = ((r2)/(r1*r3));
    // }

    trailer_theta_dot = omega_2;

    beta_dot = velocity*(((r2)/(r1*r3)) - (1/r1));

    // Special case when alpha equals zero
    if(alpha==0){
        va = velocity;
        psi = 0;
        phi = M_PI_2 - (fabs(q_current[3]) - M_PI_2);
        hitch_turn_radius_zero_alpha = trailer_wheelbase/sin(phi);
        trailer_theta_dot = va/hitch_turn_radius_zero_alpha;
        beta_dot = trailer_theta_dot;
        vb = va*fabs(cos(psi-q_current[3]));

        if(sin(phi)==0){
            beta_dot = 0;
            trailer_theta_dot = 0;
        }
    }

    

    q_dot[0] = vb*cos(q_current[2]);
    q_dot[1] = vb*sin(q_current[2]);
    q_dot[2] = velocity*((r2)/(r1*r3));
    q_dot[3] = velocity*(((r2)/(r1*r3)) - (1/r1));

    // std::cout << "q-dot called" << std::endl;

    return q_dot;
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

    std::vector<double> q_next(6,0);

    q_next[0] = q_current[0] + ((k1[0]/6)+(k2[0]/3)+(k3[0]/3)+(k4[0]/6))*timestep;
    q_next[1] = q_current[1] + ((k1[1]/6)+(k2[1]/3)+(k3[1]/3)+(k4[1]/6))*timestep;
    q_next[2] = q_current[2] + ((k1[2]/6)+(k2[2]/3)+(k3[2]/3)+(k4[2]/6))*timestep;
    q_next[3] = q_current[3] + ((k1[3]/6)+(k2[3]/3)+(k3[3]/3)+(k4[3]/6))*timestep;

    q_next[2] = wrap_angle(q_next[2]);
    q_next[3] = wrap_angle(q_next[3]);

    // For every calculated future state, also calculate the axle center of the tractor
    get_tractor_axle_center(q_next);

    // q_next[4] = q_next[0] + trailer_wheelbase*cos(q_next[3]) + tractor_m*cos(M_PI - q_next[3] + q_next[2]);
    // q_next[5] = q_next[1] + trailer_wheelbase*sin(q_next[3]) + tractor_m*sin(M_PI - q_next[3] + q_next[2]);

    if(std::isnan(q_next[0])){
        std::cout << "The integrated step is nan!" << std::endl;
        std::cout << "The previous state x is: " << q_current[0] << std::endl;
        std::cout << "The k values are: " << k1[0] << ", " << k2[0] << ", " << k3[0] << ", " << k4[0] << std::endl;
        std::cout << "alpha: " << alpha << std::endl;
        std::cout << "velocity: " << velocity << std::endl;

    }

    return q_next;

}

/*This function takes in the current state, the direction of travel and the entire piecewise linear path. It iterates through
all the segments of the path and returns the correct intersection point based on the direction of travel*/
static bool get_intersection_point_along_piecewise_linear(
    const std::vector<double>& q_current, 
    const std::vector<std::vector<double>>& piecewise_linear, 
    const bool& is_forward,
    std::vector<double>& intersection_point
){

    int intersection_counter = 0;
    bool intersection_detected_in_prev_segment = false;
    intersection_point[0] = q_current[0];
    intersection_point[1] = q_current[1];

    std::vector<double> previous_intersection_point(2,0);

    // std::cout << piecewise_linear.size() << std::endl;


    for (int i=0; i<(piecewise_linear.size()-1); i++){

        if(intersection_detected_in_prev_segment && find_intersection_point(q_current, piecewise_linear, i, i+1, is_forward, intersection_point)){
            // std::cout << "Found intersection in segment " << i << std::endl;
            return true;
        };

        if(intersection_detected_in_prev_segment && (!find_intersection_point(q_current, piecewise_linear, i, i+1, is_forward, intersection_point))){
            // std::cout << "Found intersection in segment " << i-1 << std::endl;
            intersection_point = previous_intersection_point;
            return true;
        };

        if(find_intersection_point(q_current, piecewise_linear, i, i+1, is_forward, intersection_point)){
            intersection_detected_in_prev_segment = true;
            previous_intersection_point = intersection_point;
        };

        if(find_intersection_point(q_current, piecewise_linear, i, i+1, is_forward, intersection_point) && (i==(piecewise_linear.size()-2))){
            // std::cout << "Entered here and returning true!!!" << std::endl;
            return true;
        };
    }

    return false;
}

std::vector<std::vector<double>> segment_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> segment,
    bool is_forward
){
    std::cout << "Received segment start X: " << segment[0][0] << std::endl;
    std::cout << "Received segment start Y: " << segment[0][1] << std::endl;
    std::cout << "Received segment goal X: " << segment[1][0] << std::endl;
    std::cout << "Received segment goal Y: " << segment[1][1] << std::endl;
    std::cout << "Received direction of travel: " << is_forward << std::endl;


    // Set velocity sign based on direction
    if(is_forward){
        velocity = fabs(velocity);
    }
    else{
        velocity = -1*fabs(velocity);
    }

    // Initialize variables for simulation
    double beta_desired;
    double beta_e;
    double alpha_e;
    double alpha;
    std::vector<double> q_next;


    // Create the vector of vectors which stores the trajectory
    std::vector<std::vector<double>> trajectory;

    // Constants
    float beta_prop_gain = 0;

    // Simulation time step
    double timestep = 0.001;

    // Check if inputs are valid
    if ((segment.size()<2) || (segment.size()>2)){
        throw std::runtime_error("Size of segment in segmentwise simulator is less than or greater than 2 which is invalid. Size should exactly equal two! Segment must be defined by a pair of points.");
    }

    // Initialize the intersection point to be at the center of the rear axle, a controlled junk value
    std::vector<double> intersection_point{q_init[0], q_init[1]};

    // Set the current state to the initial state of the trailer
    std::vector<double> q_current=q_init;

    // Run the simulation till the intersection point of the look-ahead circle and piecewise linear path
    // reaches the last point along the piecewise linear path

    bool found_intersection_flag=true;

    int while_loop_counter = 0;

    while(
        !(check_double_equal(intersection_point[0], segment[segment.size()-1][0]) &&
    check_double_equal(intersection_point[1], segment[segment.size()-1][1]))
    ){

        // std::cout << "Simulating" << std::endl;

        // For every line segment in the piecewise linear path, search for intersection points
        // Find the intersection points of the lookahead circle and piecewise linear path
        found_intersection_flag = get_intersection_point_along_piecewise_linear(q_current, segment, is_forward, intersection_point);

        // If the new segment starts and the intersection cannot be found (precision issues for double and a result of 
        // termination critera of last segment), inflate the lookaheads by 1%
        if(while_loop_counter<=200 && (!found_intersection_flag)){
            std::cout << "Inflation required!" << std::endl;
            if(is_forward){
                forward_lookahead_radius = forward_lookahead_radius*1.01;
                found_intersection_flag = get_intersection_point_along_piecewise_linear(q_current, segment, is_forward, intersection_point);
                forward_lookahead_radius = forward_lookahead_radius/1.01;
            }
            else{
                backward_lookahead_radius = backward_lookahead_radius*1.05;
                // std::cout << "Inflated backward lookahead radius: " << backward_lookahead_radius << std::endl;
                // std::cout << "Current trailer X: " << q_current[0] << "Current Trailer Y: " << q_current[1] << std::endl;
                found_intersection_flag = get_intersection_point_along_piecewise_linear(q_current, segment, is_forward, intersection_point);
                // std::cout << "Dist to segment start: " << sqrt(pow(segment[0][0] - q_current[0],2)+pow(segment[0][1] - q_current[1],2)) << std::endl;
                backward_lookahead_radius = backward_lookahead_radius/1.05;
            }
        }

        // if((while_loop_counter%20000)==0){
        //     std::cout << "Intersection X: " << intersection_point[0] << ", Intersection Y: " << intersection_point[1] << "   ";
        // }
        // if(while_loop_counter==0){
        //     // Need to add condition that prevents intersection points outside the segment
        //     std::cout << "While loop counter: " << while_loop_counter << std::endl;
        //     std::cout << "Intersection X: " << intersection_point[0] << ", Intersection Y: " << intersection_point[1] << std::endl;
        // }
        // std::cout << "Was an intersection found?: " << found_intersection_flag << std::endl;

        if(!found_intersection_flag){
            break;
        }

        //TODO: Add a check which evaluates whether the intersection point at this time actually lies on one of the look-ahead
        // circles. If it doesn't, then the get_intersection function has not returned a proper intersection point, or it has
        // not updated the intersection point from the initalization of the intersection point with the trailers axle center


        // Here, use the forward/reverse flag in the piece-wise linear input to compute control inputs
        if(!is_forward){

            // beta desired works only for the reversing simulation now, need to create a different approach for the forward motion
            beta_desired = get_beta_desired(q_current, intersection_point, is_forward);
            // if((while_loop_counter%20000)==0){
            //     std::cout << "Beta desired: " << beta_desired << "   beta_now: " << q_current[3] << "   ";
            // }

            // Add the proportional gain beta
            beta_e = beta_desired + beta_prop_gain*(beta_desired - q_current[3]);

            // Get the value of alpha_e from beta_e using the pre-compensation link
            alpha_e = get_alpha_e(beta_desired);
            // if((while_loop_counter%20000)==0){
            //     std::cout << "Alpha_e: " << alpha_e << "    ";
            // }

            // Use alpha_e to compute steering angle input
            alpha = clip_to_alpha_e(wrap_angle(alpha_e - get_gain(beta_e, alpha_e, velocity)*(q_current[3] - beta_e)));
            // if((while_loop_counter%20000)==0){
            //     // std::cout << "Alpha: " << alpha << std::endl;
            // }

            // Integrate motion through 
            q_next = rk4_integrator(alpha, velocity, q_current, timestep);

        }
        else{

            // Use alpha_e to compute steering angle input
            alpha = clip_to_alpha_e(get_alpha_for_forward_motion(intersection_point, q_current));
            alpha = 1*alpha;

            // std::cout << "In forward and alpha is: " << alpha << std::endl;

            if (std::isnan(alpha)){
                std::cout << "Alpha was nan" << std::endl;
            }

            // Integrate motion through 
            q_next = rk4_integrator(alpha, velocity, q_current, timestep);

        }

        // if((while_loop_counter%1)==0){
        //     std::cout << "State X: " << q_next[0] << ", State Y: " << q_next[1] << std::endl;
        // }

        q_current = q_next;

        // Append the next state into the trajectory
        q_next.push_back(alpha);
        trajectory.push_back(q_next);

        while_loop_counter++;
    }

    std::cout << "Simulation complete!" << std::endl;
    if(!found_intersection_flag){
        std::cout << "Simulation cut off since no intersection point found" << std::endl;
        // std::cout << "X value of last q_next state" << q_next[0] << std::endl;
        // std::cout << "Terminated intersection point is"
    }
    // std::cout<<"Trajectory size: "<<trajectory.size()<<std::endl;
    // if(trajectory.size()==0){
    //     std::cout << "Trajectory size is zero!" << std::endl;
    //     std::cout << "Segment X0: " << segment[0][0] << "Segment Y0: " << segment[0][1] << std::endl;
    //     std::cout << "Segment X1: " << segment[1][0] << "Segment Y1: " << segment[1][1] << std::endl;
    // }
    return trajectory;
}


std::vector<std::vector<double>> forward_simulator(
    std::vector<double> q_init,
    std::vector<std::vector<double>> piecewise_linear
){

    // Create a vector to store flags for whether a segment was tracked or not
    // std::vector<bool> tracking_history(piecewise_linear.size(),false);

    // Temporary variables for trajectory plotting
    std::vector<double> x,y;

    // Create the vector of vectors which stores the trajectory
    std::vector<std::vector<double>> final_trajectory;
    std::vector<std::vector<double>> temp_trajectory;

    // Add the initial configuration of the tractor-trailer into the trajectory
    final_trajectory.push_back(q_init);

    // Check if inputs are valid
    if (piecewise_linear.size()<2){
        throw std::runtime_error("Piecewise linear input path is of insufficient size of elements!");
    }

    // Set the current state to the initial state of the trailer
    std::vector<double> q_current=q_init;

    // Run the simulation till the intersection point of the look-ahead circle and piecewise linear path
    // reaches the last point along the piecewise linear path

    bool found_intersection_flag=true;

    // Current sim segment is different from the piecewise linear since it does not store the direction boolean flag
    // and is only one segment of the piecewise linear
    std::vector<std::vector<double>> current_sim_segment(2,std::vector<double>(2,0));

    for (int i=0; i<piecewise_linear.size()-1; i++){

        std::cout << "Simulating segment: " << i << std::endl;

        // Select the correct segment from the piecewise linear path for simulation
        current_sim_segment[0][0] = piecewise_linear[i][0];
        current_sim_segment[0][1] = piecewise_linear[i][1];
        current_sim_segment[1][0] = piecewise_linear[i+1][0];
        current_sim_segment[1][1] = piecewise_linear[i+1][1];

        if(i==0){
            final_trajectory = segment_simulator(q_current, current_sim_segment, piecewise_linear[i+1][2]);
        }
        else{
            temp_trajectory = segment_simulator(q_current, current_sim_segment, piecewise_linear[i+1][2]);
            final_trajectory.insert(final_trajectory.end(), temp_trajectory.begin(), temp_trajectory.end());
        }

        q_current = final_trajectory[final_trajectory.size()-1];

    }

    // Copying x and y from trajectory for visulization
    for (int i=0; i<final_trajectory.size(); i++){
        x.push_back(final_trajectory[i][0]);
        y.push_back(final_trajectory[i][1]);
    }

    // plt::plot(x, y, "k-");

    // show plots
    // plt::show();

    return final_trajectory;

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
    std::vector<double> q_current{0,0,M_PI_2,0,0,0};
    std::vector<std::vector<double>> piecewise_linear(2, std::vector<double>(2,0));
    int id_start = 0;
    int id_goal = 1;
    bool is_forward =false;
    std::vector<double> intersection_point(2,0);

    piecewise_linear[0][0] = 0;
    piecewise_linear[0][1] = -1.9;
    piecewise_linear[1][0] = -5;
    piecewise_linear[1][1] = -1.9;
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
    // plt::plot(time, thetas, "b-");
    // plt::subplot(1,2,4);
    // plt::plot(time, betas, "g-");
    plt::plot(x, y, "b-");
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
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e, velocity) << std::endl;

    alpha_e = -0.1;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e, velocity) << std::endl;

    alpha_e = 0.2;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e, velocity) << std::endl;

    alpha_e = -0.2;
    beta_e = get_beta_e_given_alpha(alpha_e);
    std::cout << "Value of gain for beta_e: " << beta_e << " and alpha_e is: " << alpha_e << get_gain(beta_e, alpha_e, velocity) << std::endl;

}

static void test_get_alpha_for_forward_motion(){

    std::vector<double> q_init(6,0);
    std::vector<double> intersection_point(2,0);

    q_init[0] = 0;
    q_init[1] = 0;
    q_init[2] = M_PI_2;
    q_init[3] = M_PI;
    get_tractor_axle_center(q_init);
    std::cout << "X of tractor: " << q_init[4] << "   Y of tractor: " << q_init[5] << std::endl;

    intersection_point[0] = 0;
    intersection_point[1] = 2;
    std::cout << "Alpha is: " << get_alpha_for_forward_motion(intersection_point, q_init) << std::endl;

    intersection_point[0] = -0.2;
    intersection_point[1] = 2;
    std::cout << "Alpha is: " << get_alpha_for_forward_motion(intersection_point, q_init) << std::endl;

    intersection_point[0] = 0.2;
    intersection_point[1] = 2;
    std::cout << "Alpha is: " << get_alpha_for_forward_motion(intersection_point, q_init) << std::endl;

}

static void test_forward_simulator_reversing(){

    // Create a fake starting condition for the tractor and trailer
    std::vector<double> q_init(6,0);
    q_init[0] = 0;
    q_init[1] = 0;
    q_init[2] = M_PI_2;
    q_init[3] = 2.2;
    get_tractor_axle_center(q_init);

    // Create a fake piecewise linear path
    std::vector<std::vector<double>> path(4, std::vector<double>(3,0));
    path[0][0] = 0;
    path[0][1] = 0;
    path[0][2] = 0;

    path[1][0] = 0;
    path[1][1] = -5;
    path[1][2] = 0;

    path[2][0] = 7;
    path[2][1] = -6;
    path[2][2] = 0;

    path[3][0] = 9;
    path[3][1] = 0;
    path[3][2] = 0;

    std::vector<double> path_x, path_y;

    path_x.push_back(path[0][0]);
    path_x.push_back(path[1][0]);
    path_x.push_back(path[2][0]);
    path_x.push_back(path[3][0]);

    path_y.push_back(path[0][1]);
    path_y.push_back(path[1][1]);
    path_y.push_back(path[2][1]);
    path_y.push_back(path[3][1]);

    plt::plot(path_x, path_y, "k-");

    // show plots
    plt::show();



    std::cout << "Size of test path is: " << path.size() << std::endl;

    // Boolean flag for forward or backward motion
    // bool is_forward = false;

    // Get the simulated trajectory
    std::vector<std::vector<double>> traj = forward_simulator(q_init, path);

    // Save the computed trajectory into a text file
	std::ofstream test_trajectory_file;
	test_trajectory_file.open("../output/TestReversingTrajectory.txt", std::ios::trunc); // Creates new or replaces existing file
	if (!test_trajectory_file.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	test_trajectory_file << "This is a test trajectory computed for unit testing" << std::endl; // Description
	test_trajectory_file << "Forward lookahead radius: " << forward_lookahead_radius << "Backward lookahead radius: " << 
    backward_lookahead_radius << "Tractor Wheelbase: " << tractor_wheelbase << "Trailer Wheelbase: " << trailer_wheelbase << 
    "Tractor hitch offset: " << tractor_m << "Velocity" << velocity << std::endl; // Tractor and Trailer Parameters

    test_trajectory_file << std::endl;

    test_trajectory_file << "Piecewise Linear Path" << std::endl;

    for (auto& segment : path){
        test_trajectory_file << segment[0] << ", " << segment[1] << std::endl;
    }

    for (auto& state : traj){
        test_trajectory_file << state[0] << " " << state[1] << " " << state[2] << " " <<
        (M_PI - state[3]) << " " << state[4] << " " << state[5] << " " << state[6] << std::endl;
    }

	test_trajectory_file << "End of Trajectory Sequence" << std::endl;

    // Function to evaluate the correctness of the get_gain function which is defined in controller.cpp

}

static void test_forward_simulator_forward_motion(){

    // Create a fake starting condition for the tractor and trailer
    std::vector<double> q_init(6,0);
    q_init[0] = 0;
    q_init[1] = 0;
    q_init[2] = M_PI_2;
    q_init[3] = 2.2;
    get_tractor_axle_center(q_init);

    std::cout << "Tractor axle center X: " << q_init[4] << "Tractor axle center Y: " << q_init[5] << std::endl;

    // Create a fake piecewise linear path
    std::vector<std::vector<double>> path(5, std::vector<double>(3,0));
    path[0][0] = 0;
    path[0][1] = 0;
    path[0][2] = 1;

    path[1][0] = 0;
    path[1][1] = 5;
    path[1][2] = 1;

    path[2][0] = 7;
    path[2][1] = 6;
    path[2][2] = 1;

    path[3][0] = 9;
    path[3][1] = 0;
    path[3][2] = 1;

    path[4][0] = 2;
    path[4][1] = 0;
    path[4][2] = 1;

    std::vector<double> path_x, path_y;

    path_x.push_back(path[0][0]);
    path_x.push_back(path[1][0]);
    path_x.push_back(path[2][0]);
    path_x.push_back(path[3][0]);
    path_x.push_back(path[4][0]);


    path_y.push_back(path[0][1]);
    path_y.push_back(path[1][1]);
    path_y.push_back(path[2][1]);
    path_y.push_back(path[3][1]);
    path_y.push_back(path[4][1]);


    plt::plot(path_x, path_y, "k-");

    // show plots
    plt::show();



    std::cout << "Size of test path is: " << path.size() << std::endl;

    // Boolean flag for forward or backward motion
    // bool is_forward = false;

    // Get the simulated trajectory
    std::vector<std::vector<double>> traj = forward_simulator(q_init, path);

    // Save the computed trajectory into a text file
	std::ofstream test_trajectory_file;
	test_trajectory_file.open("../output/TestForwardTrajectory.txt", std::ios::trunc); // Creates new or replaces existing file
	if (!test_trajectory_file.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	test_trajectory_file << "This is a test trajectory computed for unit testing" << std::endl; // Description
	test_trajectory_file << "Forward lookahead radius: " << forward_lookahead_radius << "Backward lookahead radius: " << 
    backward_lookahead_radius << "Tractor Wheelbase: " << tractor_wheelbase << "Trailer Wheelbase: " << trailer_wheelbase << 
    "Tractor hitch offset: " << tractor_m << "Velocity" << velocity << std::endl; // Tractor and Trailer Parameters

    test_trajectory_file << std::endl;

    test_trajectory_file << "Piecewise Linear Path" << std::endl;

    for (auto& segment : path){
        test_trajectory_file << segment[0] << ", " << segment[1] << std::endl;
    }

    for (auto& state : traj){
        test_trajectory_file << state[0] << " " << state[1] << " " << state[2] << " " <<
        (M_PI - state[3]) << " " << state[4] << " " << state[5] << " " << state[6] << std::endl;
    }

	test_trajectory_file << "End of Trajectory Sequence" << std::endl;

    // Function to evaluate the correctness of the get_gain function which is defined in controller.cpp

}


static void test_forward_simulator_mixed_path(){

    // Create a fake starting condition for the tractor and trailer
    std::vector<double> q_init(6,0);
    q_init[0] = 0;
    q_init[1] = 0;
    q_init[2] = M_PI_2;
    q_init[3] = 2.2;
    get_tractor_axle_center(q_init);

    std::cout << "Tractor axle center X: " << q_init[4] << "Tractor axle center Y: " << q_init[5] << std::endl;

    // Create a fake piecewise linear path
    std::vector<std::vector<double>> path(5, std::vector<double>(3,0));
    path[0][0] = 0;
    path[0][1] = 0;
    path[0][2] = 1;

    path[1][0] = 0;
    path[1][1] = 15;
    path[1][2] = 1;

    path[2][0] = 1;
    path[2][1] = 3;
    path[2][2] = 0;

    path[3][0] = 9;
    path[3][1] = 6;
    path[3][2] = 0;

    path[4][0] = 12;
    path[4][1] = 14;
    path[4][2] = 0;

    std::vector<double> path_x, path_y;

    path_x.push_back(path[0][0]);
    path_x.push_back(path[1][0]);
    path_x.push_back(path[2][0]);
    path_x.push_back(path[3][0]);
    path_x.push_back(path[4][0]);

    path_y.push_back(path[0][1]);
    path_y.push_back(path[1][1]);
    path_y.push_back(path[2][1]);
    path_y.push_back(path[3][1]);
    path_y.push_back(path[4][1]);


    plt::plot(path_x, path_y, "k-");

    // show plots
    plt::show();



    std::cout << "Size of test path is: " << path.size() << std::endl;

    // Boolean flag for forward or backward motion
    // bool is_forward = false;

    // Get the simulated trajectory
    std::vector<std::vector<double>> traj = forward_simulator(q_init, path);

    // Save the computed trajectory into a text file
	std::ofstream test_trajectory_file;
	test_trajectory_file.open("../output/TestMixedTrajectory.txt", std::ios::trunc); // Creates new or replaces existing file
	if (!test_trajectory_file.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	test_trajectory_file << "This is a test trajectory computed for unit testing" << std::endl; // Description
	test_trajectory_file << "Forward lookahead radius: " << forward_lookahead_radius << "Backward lookahead radius: " << 
    backward_lookahead_radius << "Tractor Wheelbase: " << tractor_wheelbase << "Trailer Wheelbase: " << trailer_wheelbase << 
    "Tractor hitch offset: " << tractor_m << "Velocity" << velocity << std::endl; // Tractor and Trailer Parameters

    test_trajectory_file << std::endl;

    test_trajectory_file << "Piecewise Linear Path" << std::endl;

    for (auto& segment : path){
        test_trajectory_file << segment[0] << " " << segment[1] << std::endl;
    }

    for (auto& state : traj){
        test_trajectory_file << state[0] << " " << state[1] << " " << state[2] << " " <<
        (M_PI - state[3]) << " " << state[4] << " " << state[5] << " " << state[6] << std::endl;
    }

	test_trajectory_file << "End of Trajectory Sequence" << std::endl;

    // Function to evaluate the correctness of the get_gain function which is defined in controller.cpp

}

/////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// Main //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////


// int main(){

//     // test_find_intersection_point();

//     // test_get_beta_desired();

//     // test_get_alpha_e();

//     // test_get_gain();

//     // gain_scheduler();

//     // test_q_dot();

//     // test_rk4_integration_function();

//     // test_forward_simulator_reversing();

//     // test_forward_simulator_forward_motion();

//     test_forward_simulator_mixed_path();

//     // test_get_alpha_for_forward_motion();

//     // test_wrap_angle();

//     return 0;

// }

//