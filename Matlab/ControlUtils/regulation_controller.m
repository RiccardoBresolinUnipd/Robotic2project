function [v, omega] = regulation_controller(x,y,theta, target, gains)

% Errors in Robot Coordinates
rho     = sqrt((x - target.x)^2 + (y - target.y)^2);
gamma   = atan2(y - target.y, x - target.x) + pi - (theta );
delta   = gamma + (theta- target.theta);

% Simple Regulation Control
v     = gains.kv *rho* cos(gamma) ;
omega = gains.kw * gamma + (gains.kv*sin(gamma)*cos(gamma)/gamma)*(gamma + gains.kd * delta);

end
