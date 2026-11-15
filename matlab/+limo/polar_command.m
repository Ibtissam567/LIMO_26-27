function u = polar_command(x, goal, g)
%POLAR_COMMAND Polar control from last year's LIMO lab.
%
%   u = limo.polar_command(x, goal, g)
%
%   x    - plant state of limo.plant_dynamics. Only X, Y and psi are used.
%   goal - [x_star; y_star; theta_star]
%   g    - gains: k_rho, k_alpha, k_beta, v_max, w_max, stop
%
%   u = [v; omega] for limo.simulate_gazebo in 'cmd_vel' mode.
%   v = sens * k_rho * rho
%   omega = sens * (k_alpha * alpha + k_beta * beta)
%   sens is -1 when the goal is behind the robot.

dx = goal(1) - x(1);
dy = goal(2) - x(2);
rho = hypot(dx, dy);

if rho < g.stop
    u = [0; 0];
    return
end

direction = atan2(dy, dx);
alpha = wrap_to_pi(direction - x(3));
beta = wrap_to_pi(goal(3) - direction);

if abs(alpha) <= pi/2
    sens = 1;
else
    sens = -1;
    alpha = wrap_to_pi(alpha - sign(alpha)*pi);
end

v = sens * g.k_rho * rho;
if rho < 0.5
    v = sign(v) * min(abs(v), 0.20*rho);
end
v = min(max(v, -g.v_max), g.v_max);

omega = sens * (g.k_alpha*alpha + g.k_beta*beta);
omega = min(max(omega, -g.w_max), g.w_max);

u = [v; omega];

end


function angle = wrap_to_pi(angle)
angle = atan2(sin(angle), cos(angle));
end
