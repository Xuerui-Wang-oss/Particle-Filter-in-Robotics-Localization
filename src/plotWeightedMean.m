%% plotWeightedMean.m
% Plot weighted-mean trajectory vs best-particle trajectory vs truth
% Uses dataStore saved from motionControl (PF mode)

clear; close all; clc;

%% ---- Load data ----------------------------------------------------------
load('pf_500.mat');  % loads dataStore

%% ---- Parameters ---------------------------------------------------------
M = 500;  % number of particles (must match motionControl setting)

%% ---- Extract truth trajectory -------------------------------------------
t_truth = dataStore.truthPose(:, 1);
x_truth = dataStore.truthPose(:, 2);
y_truth = dataStore.truthPose(:, 3);

%% ---- Extract particle data and compute trajectories ---------------------
% dataStore.particles format per row:
%   [timestamp, x1,y1,th1, x2,y2,th2, ..., w1,w2,...,wM]
%   columns: 1 (time) + 3*M (particles) + M (weights) = 1 + 4*M

nSteps = size(dataStore.particles, 1);

best_traj    = zeros(nSteps, 3);  % best particle trajectory
weighted_traj = zeros(nSteps, 3);  % weighted mean trajectory
t_pf = dataStore.particles(:, 1);

for k = 1:nSteps
    row = dataStore.particles(k, :);

    % Extract particles: columns 2 to 3*M+1
    p_flat = row(2 : 3*M+1);
    particles_k = reshape(p_flat, 3, M);  % 3-by-M

    % Extract weights: columns 3*M+2 to 4*M+1
    weights_k = row(3*M+2 : 4*M+1);

    % Best particle (highest weight)
    [~, best_idx] = max(weights_k);
    best_traj(k, :) = particles_k(:, best_idx)';

    % Weighted mean
    weighted_traj(k, 1) = sum(weights_k .* particles_k(1, :));  % x
    weighted_traj(k, 2) = sum(weights_k .* particles_k(2, :));  % y
    weighted_traj(k, 3) = sum(weights_k .* particles_k(3, :));  % theta
end

%% ---- Compute RMSE -------------------------------------------------------
% Interpolate truth to PF timestamps for fair comparison
x_truth_interp = interp1(t_truth, x_truth, t_pf, 'linear', 'extrap');
y_truth_interp = interp1(t_truth, y_truth, t_pf, 'linear', 'extrap');

err_best = sqrt((best_traj(:,1) - x_truth_interp).^2 + ...
                (best_traj(:,2) - y_truth_interp).^2);
err_weighted = sqrt((weighted_traj(:,1) - x_truth_interp).^2 + ...
                    (weighted_traj(:,2) - y_truth_interp).^2);

rmse_best     = sqrt(mean(err_best.^2));
rmse_weighted = sqrt(mean(err_weighted.^2));

fprintf('RMSE (Best Particle):  %.4f m\n', rmse_best);
fprintf('RMSE (Weighted Mean):  %.4f m\n', rmse_weighted);

%% ---- Load map -----------------------------------------------------------
mapData = load('cornerMap.mat');
fields = fieldnames(mapData);
map = mapData.(fields{1});

%% ---- Plot trajectories --------------------------------------------------
figure('Position', [100, 100, 900, 700]); hold on;

% Draw map walls (hidden from legend)
for i = 1:size(map, 1)
    plot([map(i,1), map(i,3)], [map(i,2), map(i,4)], 'k-', 'LineWidth', 2, 'HandleVisibility', 'off');
end

% Plot trajectories
plot(x_truth, y_truth, 'g-', 'LineWidth', 3, 'DisplayName', 'True Trajectory');
plot(best_traj(:,1), best_traj(:,2), 'r-', 'LineWidth', 2, 'DisplayName', ...
    sprintf('Best Particle (RMSE=%.3fm)', rmse_best));
plot(weighted_traj(:,1), weighted_traj(:,2), 'b-', 'LineWidth', 2, 'DisplayName', ...
    sprintf('Weighted Mean (RMSE=%.3fm)', rmse_weighted));

% Mark start and end for True Trajectory
plot(x_truth(1), y_truth(1), 'gs', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'True Start');
plot(x_truth(end), y_truth(end), 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'True End');

% Mark start and end for Best Particle
plot(best_traj(1,1), best_traj(1,2), 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Best Start');
plot(best_traj(end,1), best_traj(end,2), 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Best End');

% Mark start and end for Weighted Mean
plot(weighted_traj(1,1), weighted_traj(1,2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Weighted Start');
plot(weighted_traj(end,1), weighted_traj(end,2), 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', 'Weighted End');

xlabel('x [m]'); ylabel('y [m]');
title('Weighted Mean vs Best Particle Trajectory');
legend('Location', 'best');
axis equal; grid on;
hold off;

