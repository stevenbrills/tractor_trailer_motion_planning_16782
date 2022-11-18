#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>
#include <ct/optcon/optcon.h>
#include "matplotlibcpp.h"
#include "planner.h"

// Forward simulation function that takes initial configuration and a piecewise linear path
// as an input

// forward lookahead radius
extern double forward_lookahead_radius;

// backward lookahead radius
extern double backward_lookahead_radius;

// tractor wheelbase
extern double tractor_wheelbase;

// trailer wheelbase
extern double trailer_wheelbase;

// Trailer hitch offset
extern double tractor_m;

// Tractor tracking velocity
extern double velocity;

namespace plt = matplotlibcpp;

// bool check_double_equal(
//     double& a,
//     double& b
// ){
//     double epsilon = 0.001;

//     if (fabs(a-b)<=epsilon){
//         return true;
//     }

//     return false;

// }


// double get_alpha_e(
//     double& beta_e
// ){

//     // Find R3
//     double r3 = (trailer_wheelbase + (tractor_m/cos(beta_e)))/(tan(beta_e));

//     // calculate R1
//     double r1 = sqrt(pow(r3,2) + pow(trailer_wheelbase,2) - pow(tractor_m,2));

//     // calculate alpha_e
//     double alpha_e = atan((tractor_wheelbase*(beta_e/fabs(beta_e)))/r1);

//     std::cout << "Alpha_e value is: " << alpha_e << std::endl;

//     return alpha_e;
// }

// A matrix for equillibrium point
double get_state_matrix(
    const double& alpha_e,
    const double& beta_e
){
    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = -1*(alpha_e/fabs(alpha_e))*atan(tractor_m/fabs(r1));

    double r2 = tractor_m/sin(psi);

    double r3 = trailer_wheelbase/sin(psi-beta_e);

    // std::cout << "Psi: " << psi << std::endl;

    // double A = -1*(((tractor_m/(tractor_wheelbase*trailer_wheelbase))*fabs(tan(alpha_e))*((1*sin(fabs(beta_e)))+(1*(cos(fabs(beta_e))/tan(psi))))));

    double A;

    // if (alpha_e>=0){
    //     A = (1/tractor_wheelbase)*tan(alpha_e)*-1*((tractor_m*cos(beta_e - psi))/(trailer_wheelbase*sin(psi)));
    // }
    // else{
    //     A = (-1/tractor_wheelbase)*tan(alpha_e)*((tractor_m*cos((-1*beta_e) - psi))/(trailer_wheelbase*sin(psi)));
    // }

    A = (1/tractor_wheelbase)*tan(alpha_e)*(((tractor_m/trailer_wheelbase)*((-1*sin(psi)*sin(beta_e) -1*cos(psi)*cos(beta_e))/(sin(psi)))));


    // Other paper
    // if (alpha_e>=0){
    //     A = (-1/(tractor_wheelbase*trailer_wheelbase))*(tractor_m*tan(alpha_e)*(-1*sin(beta_e)) + (tractor_wheelbase*cos(beta_e)));
    // }
    // else{
    //     A = 1*(-1/(tractor_wheelbase*trailer_wheelbase))*(tractor_m*tan(alpha_e)*(-1*sin(beta_e)) + (tractor_wheelbase*cos(beta_e)));
    // }

    return A;
}

// B matrix for equillibrium point
double get_input_matrix(
    const double& alpha_e,
    const double& beta_e
){

    double r1 = tractor_wheelbase/tan(alpha_e);

    double psi = -1*(alpha_e/fabs(alpha_e))*atan(tractor_m/fabs(r1));

    double r2 = tractor_m/sin(psi);

    double r3 = trailer_wheelbase/sin(psi-beta_e);


    // std::cout << "Psi: " << psi << std::endl;

    // double B = ((1/(cos(alpha_e)*cos(alpha_e)*tractor_wheelbase)) - 
    // ((tractor_m/(tractor_wheelbase*trailer_wheelbase*cos(alpha_e)*cos(alpha_e)))*
    // ((cos(beta_e))+(-1*(sin(beta_e)/tan(psi))))));

    // double B = ((1/(cos(fabs(alpha_e))*cos(fabs(alpha_e))*tractor_wheelbase)) - 
    // ((tractor_m/(tractor_wheelbase*trailer_wheelbase*cos(fabs(alpha_e))*cos(fabs(alpha_e))))*
    // (sin(fabs(beta_e) - psi)/sin(psi))));

    double B;

    // if (alpha_e>=0){
    //    B = (1/(tractor_wheelbase*pow(cos(alpha_e),2)))*(1-((tractor_m*sin(beta_e - psi))/(trailer_wheelbase*sin(psi))));
    // }
    // else{
    //    B = (-1/(tractor_wheelbase*pow(cos(alpha_e),2)))*(1-((tractor_m*sin((-1*beta_e) - psi))/(trailer_wheelbase*sin(psi))));
    // }

    // B = (1/(tractor_wheelbase*pow(cos(alpha_e),2)))*(((tractor_m*sin(psi - beta_e))/(trailer_wheelbase*sin(psi))) - 1);

    double a = (1/tractor_wheelbase)*(1/pow(cos(alpha_e),2));
    double b = (tractor_m/trailer_wheelbase)*sin(psi-beta_e)/sin(psi);


    double c = tan(alpha_e)/tractor_wheelbase;
    
    double d = (tractor_m/trailer_wheelbase)*(sin(beta_e)*tractor_m*tractor_wheelbase);

    double e = pow(sin(alpha_e),2)*pow(sin(psi),2)*(pow(r1,2) + pow(tractor_m,2));

// -1*(alpha_e/fabs(alpha_e))*

    B = -1*(c*(d/e)  + a*(b-1));
    // return b;





    // B = a*(b - 1);
    // B = (1*((pow(tan(alpha_e),2)) + 1)/(tractor_wheelbase))*(((tractor_m*sin(psi - beta_e))/(trailer_wheelbase*sin(psi))) - 1);

    // a = -1*(tan(alpha_e)*tan(alpha_e) + 1)/tractor_wheelbase;
    
    // b = -(tractor_m*sin(beta_e-psi)*(tan(alpha_e)*tan(alpha_e) + 1))/(tractor_wheelbase*trailer_wheelbase*sin(psi));


    // B = c + a*(b-1);




    // if (alpha_e<0){
    //     return -1*B;
    // }
    // return B;

    // Other paper
    // if (alpha_e>=0){
    //     B = (-1/(tractor_wheelbase*trailer_wheelbase))*(((tractor_m*cos(beta_e))/(cos(alpha_e)*cos(alpha_e)))+(trailer_wheelbase/(cos(alpha_e)*cos(alpha_e))));
    // }
    // else{
    //     B = (-1/(tractor_wheelbase*trailer_wheelbase))*(((tractor_m*cos(beta_e))/(cos(alpha_e)*cos(alpha_e)))+(trailer_wheelbase/(cos(alpha_e)*cos(alpha_e))));
    // }

    // std::cout << "B: " << B << std::endl;

    return B;
}

double get_beta_dot(
    double& alpha,
    double& beta
){

    double r1 = tractor_wheelbase/tan(alpha);

    double psi = -1*(alpha/fabs(alpha))*atan(tractor_m/fabs(r1));

    double r2 = tractor_m/sin(psi);

    double r3 = trailer_wheelbase/sin(beta-psi);

    // double beta_dot = (tan(alpha)/tractor_wheelbase)*(1 - ((tractor_m*sin(beta-psi))/(sin(psi)*trailer_wheelbase)));
    // (alpha/fabs(alpha))*

    // double beta_dot = (fabs(tan(alpha))/tractor_wheelbase)*(1 - ((tractor_m*sin(beta-psi))/(sin(psi)*trailer_wheelbase)));


    // double beta_dot = velocity*((alpha/fabs(alpha))*(1/r1) - ((-1*alpha)/fabs(alpha))*((r2)/(r1*r3)));

    double beta_dot = velocity*((1/r1) + ((r2)/(r1*r3)));

    double omega_2 = ((r2)/(r1*-1*r3));


    return -1*beta_dot;

}

double get_their_beta_dot(
    double& alpha,
    double& beta
){

    double r1 = tractor_wheelbase/tan(alpha);

    double psi = atan(tractor_m/r1);

    double beta_dot = (-1/(tractor_wheelbase*trailer_wheelbase))*(tractor_m*tan(alpha)*cos(beta) + 
    trailer_wheelbase*tan(alpha) + tractor_wheelbase*sin(beta));

    return beta_dot;

}

// Function that returns beta_e
// double get_beta_e_given_alpha(
//     double& alpha_e
// ){

//     double r1 = tractor_wheelbase/tan(alpha_e);

//     std::cout << "R1 is : " << r1 << std::endl;

//     double r2 = sqrt(pow(r1,2) + pow(tractor_m,2));

//     double psi = atan(tractor_m/fabs(r1));

//     std::cout << "R2 is: " << r2 << std::endl;

//     double r3 = sqrt(pow(r2,2) - pow(trailer_wheelbase,2));
// // 
//     std::cout << "R3 in beta_e calculation is: " << r3 << std::endl;


//     // double beta_e = atan(trailer_wheelbase/r3) + psi;

//     // double beta_e = (alpha_e/fabs(alpha_e))*(atan(tractor_m/r1) + asin(trailer_wheelbase/(sqrt(pow(r1,2) + pow(tractor_m,2)))));
//     double beta_e = (alpha_e/fabs(alpha_e))*((M_PI_2 - psi) + acos(trailer_wheelbase/r2));


//     return beta_e;

// }

// 

double get_gain(
    const double& beta_e,
    const double& alpha_e
){

    std::cout << "In get gain function" << std::endl;

    const size_t state_dim = 1;
    const size_t control_dim = 1;

    ct::core::StateMatrix<state_dim> A;
    ct::core::ControlMatrix<control_dim> B;
    ct::core::StateMatrix<state_dim> Q;
    ct::core::ControlMatrix<control_dim> R;

    A(0,0) = get_state_matrix(alpha_e, beta_e);
    B(0,0) = get_input_matrix(alpha_e, beta_e);
    Q(0,0) = 10;
    R(0,0) = 10;

    std::cout << "A mat value: " << A(0,0) << std::endl;
    std::cout << "A mat value: " << B(0,0) << std::endl;

    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;

    lqrSolver.compute(Q, R, A, B, K);

    return K(0,0);

}


void gain_scheduler(
){

    int n = 500; // number of data points
    std::vector<double> x(n),y(n);
    std::vector<double> alphas(n),betas(n);
    std::vector<double> beta_dots(n);


    // for(int i=0; i<n; ++i) {
    //     double t = 2*M_PI*i/n;
    //     x.at(i) = 16*sin(t)*sin(t)*sin(t);
    //     y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
    // }



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
    double beta_e_max = get_beta_e_given_alpha(alpha_e_max);

    // alpha_e_max = 0.2;
    // beta_e_max = 0.3;


    double alpha_e_min = -1*alpha_e_max;
    double beta_e_min = -1*beta_e_max;

    std::cout << "Maximum value of alpha_e: " << alpha_e_max << std::endl;
    std::cout << "Maximum value of beta_e: " << beta_e_max << std::endl;
    std::cout << "A mat value: " << get_state_matrix(alpha_e_max, beta_e_max) << std::endl;
    std::cout << "B mat value: " << get_input_matrix(alpha_e_max, beta_e_max) << std::endl;


    std::cout << "Minimum value of alpha_e: " << alpha_e_min << std::endl;
    std::cout << "Minimum value of beta_e: " << beta_e_min << std::endl;
    std::cout << "A mat value: " << get_state_matrix(alpha_e_min, beta_e_min) << std::endl;
    std::cout << "B mat value: " << get_input_matrix(alpha_e_min, beta_e_min) << std::endl;



    double alpha_e = 0;
    double beta_e = 0;

    ct::core::StateMatrix<state_dim> A;
    ct::core::ControlMatrix<control_dim> B;
    ct::core::StateMatrix<state_dim> Q;
    ct::core::ControlMatrix<control_dim> R;

    ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    ct::core::FeedbackMatrix<state_dim, control_dim> K;

    lqrSolver.compute(Q, R, A, B, K);

    double a;
    double b;

    // loop from -alpha_e_max to +alpha_e_max
    for(int i=0; i<n; i++){

        // a = 1*alpha_e_max;

        // b = (i/(double)(n-1))*(alpha_e_max*2);
        
        // alpha_e = (alpha_e_max) - 1*alpha_e_max*((n-1-i)/(double)(n-1));
        // alpha_e = (i/(double)(n-1))*(alpha_e_max) - 1*alpha_e_max;
        alpha_e = (i/(double)(n-1))*(2*alpha_e_max) - 1*alpha_e_max;



        // alpha_e = i;
    
        beta_e = get_beta_e_given_alpha(alpha_e);

        alphas[i] = alpha_e;
        betas[i] = beta_e;

        // if (i%49==0){
        //     std::cout << "Value of i is: " << i << std::endl;
        //     std::cout << "Alpha e value is: " << alpha_e << std::endl;
        // }

        // alpha_e = 0.1;
        // beta_e = 0.1;

        A(0,0) = get_state_matrix(alpha_e, beta_e);
        B(0,0) = get_input_matrix(alpha_e, beta_e);
        Q(0,0) = 10;
        R(0,0) = 10;

        ct::optcon::LQR<state_dim, control_dim> lqrSolver;
        ct::core::FeedbackMatrix<state_dim, control_dim> K;

        // std::cout << "A: " << std::endl << A << std::endl << std::endl;
        // std::cout << "B: " << std::endl << B << std::endl << std::endl;
        // std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
        // std::cout << "R: " << std::endl << R << std::endl << std::endl;

        lqrSolver.compute(Q, R, A, B, K);




        x[i] = alpha_e;
        y[i] = K(0,0);
        beta_dots[i] = get_beta_dot(alpha_e, beta_e);

    //    std::cout << "X-value is: " << x[i] << std::endl;
    //    std::cout << "Y-value is: " << y[i] << std::endl;

        // double t = 2*M_PI*i/n;
        // x.at(i) = 16*sin(t)*sin(t)*sin(t);
        // y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
    }

    // std::cout << "LQR gain matrix:" << std::endl << K(0,0) << std::endl;


    // , x, [](double d) { return 12.5+abs(sin(d)); }, "k-"

    // std::cout << "Sum of alpha_e values: " << sum << std::endl;

    
    plt::plot(x, y, "k-");

    // plt::plot(alphas, betas, "r-");

    // plt::plot(x, beta_dots, "r-");

    // plt::plot()

    // show plots
    plt::show();

    // for a given alpha_e, compute corresponding beta_e

    // Using beta_e and alpha_e, compute state and input matrices for the equillibrium point


    // double test = get_state_matrix(alpha_e, beta_e);

    // std::cout << "Returned value of get_state_matrix: " << test << std::endl;

    // double B = get_input_matrix(alpha_e, beta_e);












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



void beta_dot_tester(
){

    int n = 500; // number of data points
    std::vector<double> x(n),y(n);
    std::vector<double> alphas(n),betas(n);
    std::vector<double> beta_dots(n);

    std::cout << "In the gain scheduler function" << std::endl;

    // Constants
    float beta_prop_gain = 0.3;

    // calculate the extents of alpha_e
    double alpha_e_max = atan(tractor_wheelbase/sqrt(pow(trailer_wheelbase,2) - pow(tractor_m,2)));
    double beta_e_max = get_beta_e_given_alpha(alpha_e_max);

    double test_beta = 0;
    double test_constant_alpha = -0.2;

    double beta_start = M_PI;
    double beta_end = -M_PI;

    double alpha = 0;
    double beta = 0;

    double a;
    double b;

    // loop from -alpha_e_max to +alpha_e_max
    for(int i=0; i<n; i++){
        
        // alpha_e = (alpha_e_max) - 1*alpha_e_max*((n-1-i)/(double)(n-1));
        // alpha_e = (i/(double)(n-1))*(alpha_e_max) - 1*alpha_e_max;
        // alpha_e = (i/(double)(n-1))*(2*alpha_e_max) - 1*alpha_e_max;

        beta = (i/(double)(n-1))*(beta_end - beta_start) + 1*beta_start;
    
        // beta_e = get_beta_e_given_alpha(alpha_e);

        alphas[i] = beta;
        betas[i] = get_beta_dot(test_constant_alpha, beta);


        // x[i] = alpha_e;
        // y[i] = K(0,0);
        // beta_dots[i] = get_their_beta_dot(alpha_e, beta_e);

    }

    // double test_beta_e = -1*M_PI - get_beta_e_given_alpha(test_constant_alpha);
    double test_beta_e = get_beta_e_given_alpha(test_constant_alpha);


    std::cout << "Beta_e for the constant alpha is: " << test_beta_e << std::endl;

    std::cout << "Beta dot for beta_e and test alpha is: " << get_beta_dot(test_constant_alpha, test_beta_e) << std::endl;

    
    plt::plot(x, y, "k-");

    // plt::plot(alphas, betas, "r-");

    // plt::plot(x, beta_dots, "r-");

    // plt::plot()

    // show plots
    plt::show();

}


// int main(){

//     // test_find_intersection_point();

//     // test_get_beta_desired();

//     // test_get_alpha_e();

//     // Prepare data.
//     // int n = 5000; // number of data points
//     // std::vector<double> x(n),y(n);
//     // for(int i=0; i<n; ++i) {
//     //     double t = 2*M_PI*i/n;
//     //     x.at(i) = 16*sin(t)*sin(t)*sin(t);
//     //     y.at(i) = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t);
//     // }

//     // plot() takes an arbitrary number of (x,y,format)-triples.
//     // x must be iterable (that is, anything providing begin(x) and end(x)),
//     // y must either be callable (providing operator() const) or iterable.
//     // plt::plot(x, y, "r-");//, x, [](double d) { return 12.5+abs(sin(d)); }, "k-");


//     // // show plots
//     // plt::show();

//     gain_scheduler();

//     // beta_dot_tester();

//     return 0;

// }

//