
Xd = [1,1];
Xs = [0,0];

look_ahead_radius = 2;

vehicle_length = 0.5;
theta = pi/2;

alpha_desired = pure_pursuit_func(Xd,Xs,look_ahead_radius,vehicle_length)