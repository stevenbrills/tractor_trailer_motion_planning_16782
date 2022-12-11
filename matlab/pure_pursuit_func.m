function alpha_desired = pure_pursuit_func(Xd,Xs,look_ahead_radius,vehicle_length)

% Xd -> desired coordinates of the robot
% Xs -> start coordinates
% theta -> vehicle orientation;

if norm(Xd-Xs) <= look_ahead_radius
    alpha_desired = nan;        % when lookahead circle reaches the point.
%     disp("waypoint reached")
    return;
end

x = Xd(1) - Xs(1);
y = Xd(2) - Xs(2);

l = sqrt(x^2+y^2);
r = l^2/(2*abs(x));

alpha_desired = -1*(x/abs(x))*atan2(vehicle_length,r);

end