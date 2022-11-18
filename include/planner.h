#ifndef TRACTOR_TRAILER_PLANNER_HEADER_GUARD
#define TRACTOR_TRAILER_PLANNER_HEADER_GUARD



double get_gain(
    const double& beta_e,
    const double& alpha_e
);

void gain_scheduler();

double get_beta_e_given_alpha(
    double& alpha_e
);

#endif