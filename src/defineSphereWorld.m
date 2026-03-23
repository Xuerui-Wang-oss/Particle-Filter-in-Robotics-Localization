%% defineSphereWorld.m
% Define a sphere world that over-approximates the rectangular obstacles
% in hw5aMap.txt, and plot the result.

%% Obstacle definitions from hw5aMap.txt
% Obstacle 1: x in [1.991, 3.009], y in [-3.050, -1.990]
obs1_center = [(1.991+3.009)/2, (-3.050+-1.990)/2];  % [2.500, -2.520]
obs1_half_diag = norm([(3.009-1.991)/2, (-1.990-(-3.050))/2]);  % ~0.735

% Obstacle 2: x in [-0.957, 1.009], y in [1.990, 3.990]
obs2_center = [(-0.957+1.009)/2, (1.990+3.990)/2];   % [0.026, 2.990]
obs2_half_diag = norm([(1.009-(-0.957))/2, (3.990-1.990)/2]);  % ~1.402

% Obstacle 3: x in [-3.991, -1.957], y in [-3.010, -0.990]
obs3_center = [(-3.991+-1.957)/2, (-3.010+-0.990)/2]; % [-2.974, -2.000]
obs3_half_diag = norm([(-1.957-(-3.991))/2, (-0.990-(-3.010))/2]);  % ~1.433

% Add a small safety margin to each radius
margin = 0.1;

% Workspace boundary sphere: centered at origin, large enough to contain everything
boundary_center = [0, 0];
boundary_radius = 7;

%% Load sphere world map from file (also usable by SimulatorGUI)
sphereMap = load('hw5aSphere.txt');

fprintf('Sphere world map:\n');
fprintf('  Boundary:   center = (%.3f, %.3f), radius = %.3f\n', sphereMap(1,:));
fprintf('  Obstacle 1: center = (%.3f, %.3f), radius = %.3f\n', sphereMap(2,:));
fprintf('  Obstacle 2: center = (%.3f, %.3f), radius = %.3f\n', sphereMap(3,:));
fprintf('  Obstacle 3: center = (%.3f, %.3f), radius = %.3f\n', sphereMap(4,:));

%% Plot the sphere world with original rectangular obstacles
figure; hold on; axis equal; grid on;
title('Sphere World Over-Approximation of hw5aMap');
xlabel('x'); ylabel('y');

% Draw workspace boundary
theta = linspace(0, 2*pi, 200);
plot(boundary_center(1) + boundary_radius*cos(theta), ...
     boundary_center(2) + boundary_radius*sin(theta), 'k-', 'LineWidth', 2);

% Draw rectangular obstacles (original)
rectangles = {
    [1.991, -3.050, 3.009-1.991, -1.990-(-3.050)];
    [-0.957, 1.990, 1.009-(-0.957), 3.990-1.990];
    [-3.991, -3.010, -1.957-(-3.991), -0.990-(-3.010)];
};
for i = 1:3
    r = rectangles{i};
    rectangle('Position', r, 'EdgeColor', 'b', 'LineWidth', 1.5, 'LineStyle', '--');
end

% Draw sphere (circle) approximations
colors = {'r', 'g', 'm'};
for i = 2:size(sphereMap, 1)
    cx = sphereMap(i, 1);
    cy = sphereMap(i, 2);
    cr = sphereMap(i, 3);
    plot(cx + cr*cos(theta), cy + cr*sin(theta), colors{i-1}, 'LineWidth', 2);
    plot(cx, cy, [colors{i-1} 'x'], 'MarkerSize', 10, 'LineWidth', 2);
end

% Mark goal
plot(0, 0, 'p', 'MarkerSize', 15, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
text(0.3, 0.3, 'Goal', 'FontSize', 12);

legend('Boundary', 'Rect Obs 1', 'Rect Obs 2', 'Rect Obs 3', ...
       'Sphere Obs 1', '', 'Sphere Obs 2', '', 'Sphere Obs 3', '', ...
       'Goal', 'Location', 'best');

xlim([-8, 8]); ylim([-8, 8]);
hold off;

%% Generate potential field plot
goal = [0, 0];
c_att = 15;
c_rep = 8;
Q = 5;

x = linspace(-boundary_radius, boundary_radius, 100);
y = linspace(-boundary_radius, boundary_radius, 100);
[X, Y] = meshgrid(x, y);
points = [X(:), Y(:)];

TestSphereWorldPot(sphereMap, goal, c_att, c_rep, Q, points);
