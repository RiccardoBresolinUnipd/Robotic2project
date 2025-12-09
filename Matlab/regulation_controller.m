function [v, omega] = regulation_controller(x,y,theta, target, gains)

% errori in coordinate del robot
ex =  cos(theta)*(target.x - x) + sin(theta)*(target.y - y);
ey = -sin(theta)*(target.x - x) + cos(theta)*(target.y - y);
et = wrapToPi(target.theta - theta);

% controllo semplice di regolazione
v     = gains.kp * ex;
omega = gains.kw * et;

end
