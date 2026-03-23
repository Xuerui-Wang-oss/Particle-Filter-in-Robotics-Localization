clear; close all; clc;

%% ---- Parameters ---------------------------------------------------------
M = 30;               % number of particles
n_sensors = 9;        % number of depth sensors

%% ---- Create initial particle set ----------------------------------------
x_init = -1 + 2 * rand(1, M);          % x ~ U[-1, 1]
y_init = -3 + 4 * rand(1, M);          % y ~ U[-3, 1]
theta_init = (pi/2) * ones(1, M);      % theta = 90 degrees

particles_init = [x_init; y_init; theta_init];  % 3-by-M
weights_init = ones(1, M) / M;                  % uniform weights

%% ---- Define control input and measurement --------------------------------
ut = [1; 0];                             % d = 1, phi = 0
z  = 2 * ones(n_sensors, 1);            % all 9 depth measurements are 2 meters

%% ---- Define dynamics function (same as integrateOdom for one step) ------
g_func = @(x, u) [x(1) + u(1) * cos(x(3) + u(2)/2);
                   x(2) + u(1) * sin(x(3) + u(2)/2);
                   x(3) + u(2)];

%% ---- Define measurement likelihood function -----------------------------
p_func = @(x, z, Q) depthLikelihood(x, z, Q);

%% ---- Noise parameters ---------------------------------------------------
R = 0.01 * eye(3);                      % process noise covariance
Q = 0.1 * eye(n_sensors);               % measurement noise covariance

%% ---- Run one iteration of the particle filter ---------------------------
[particles_final, weights_final] = PF(particles_init, weights_init, ut, z, ...
                                       g_func, p_func, R, Q);

%% ---- Plot results -------------------------------------------------------
figure; hold on;

% Plot the wall at y = 1
plot([-3 3], [1 1], 'k-', 'LineWidth', 3, 'DisplayName', 'Wall (y=1)');

% Plot initial particles (circles)
plot(particles_init(1,:), particles_init(2,:), 'bo', 'MarkerSize', 8, ...
     'LineWidth', 1.5, 'DisplayName', 'Initial Particles');

% Plot final particles (triangles)
plot(particles_final(1,:), particles_final(2,:), 'r^', 'MarkerSize', 8, ...
     'LineWidth', 1.5, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Particles');

xlabel('x'); ylabel('y');
title('Particle Filter: Wall at y=1, d=1, \phi=0, depth=2m');
legend('Location', 'best');
grid on;
axis equal;
xlim([-3, 3]);
ylim([-5, 3]);
hold off;

fprintf('Test complete. Initial particles: %d, Final particles: %d\n', ...
    size(particles_init, 2), size(particles_final, 2));
