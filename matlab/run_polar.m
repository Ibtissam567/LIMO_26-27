%RUN_POLAR Drive the Gazebo LIMO model from A to Z with the polar law.
%
%   The robot is the model in this folder. The controller is the polar law
%   of last year's LIMO lab (fichierlimo.m). k_beta is -10, the value that
%   keeps the law stable. The -100000 currently written in fichierlimo.m
%   makes the robot spin and miss the point.

folder = fileparts(mfilename('fullpath'));
addpath(folder);

p = limo.parameters();

% A: Room 207 spawn. Z: 2.5 m straight ahead, same heading.
p.initial.pose = p.initial.room207_pose;
start = p.initial.pose;
goal = [
    start(1) + 2.5*cos(start(3))
    start(2) + 2.5*sin(start(3))
    start(3)
];

g.k_rho = 4;
g.k_alpha = 40;
g.k_beta = -10;
g.v_max = 0.30;
g.w_max = 0.80;
g.stop = 0.10;

command = @(t, x) limo.polar_command(x, goal, g);
out = limo.simulate_gazebo(command, 40, p);

final_error = hypot(out.x(end,1) - goal(1), out.x(end,2) - goal(2));
fprintf('A = (%.3f, %.3f)\n', start(1), start(2));
fprintf('Z = (%.3f, %.3f)\n', goal(1), goal(2));
fprintf('Final error = %.3f m (limit 0.10 m)\n', final_error);

figure('Name', 'Polar control A to Z');
plot(out.x(:,1), out.x(:,2), 'LineWidth', 1.5);
hold on;
plot(start(1), start(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
plot(goal(1), goal(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
axis equal;
grid on;
xlabel('X [m]');
ylabel('Y [m]');
legend('trajectory', 'A', 'Z');
title(sprintf('Final error %.3f m', final_error));
