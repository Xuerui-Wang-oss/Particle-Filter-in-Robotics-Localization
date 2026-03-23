% plotTrajectories.m - Part (d): Plot best particle trajectory,
% 5 additional particle trajectories, and true trajectory on the map

% Load saved data
pfData = load('pf_500.mat');
dataStore = pfData.dataStore;

% Load map
scriptDir = fileparts(mfilename('fullpath'));
mapData = load(fullfile(scriptDir, 'cornerMap.mat'));
fields = fieldnames(mapData);
map = mapData.(fields{1});

% Parse particle data
% Each row: [timestamp, x1,y1,th1, x2,y2,th2, ..., xM,yM,thM, w1,w2,...,wM]
particleData = dataStore.particles;
nSteps = size(particleData, 1);
M = (size(particleData, 2) - 1) / 4;  % 3*M states + M weights + 1 timestamp

% Pick 5 fixed particle indices (evenly spaced, excluding index 1)
rng(42);  % reproducible
extraIdx = randperm(M, 5);  % 5 random fixed indices

% Extract trajectories
best_traj = zeros(nSteps, 2);          % [x, y] of best particle per step
extra_traj = zeros(nSteps, 2, 5);      % 5 fixed particle trajectories

for t = 1:nSteps
    row = particleData(t, :);
    states = reshape(row(2 : 3*M+1), 3, M);   % 3-by-M
    weights = row(3*M+2 : end);                 % 1-by-M

    % Best particle (highest weight at this step)
    [~, bestIdx] = max(weights);
    best_traj(t, :) = states(1:2, bestIdx)';

    % 5 fixed-index particles
    for k = 1:5
        extra_traj(t, :, k) = states(1:2, extraIdx(k))';
    end
end

% True trajectory
trueTraj = dataStore.truthPose(:, 2:3);

%% Plot
figure; hold on;

% Draw map walls
for i = 1:size(map, 1)
    plot([map(i,1) map(i,3)], [map(i,2) map(i,4)], 'k-', 'LineWidth', 2, ...
         'HandleVisibility', 'off');
end

% Plot true trajectory
plot(trueTraj(:,1), trueTraj(:,2), 'g-', 'LineWidth', 2, ...
     'DisplayName', 'True Trajectory');

% Plot best particle trajectory
plot(best_traj(:,1), best_traj(:,2), 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Best Particle Trajectory');
plot(best_traj(1,1), best_traj(1,2), 'rs', 'MarkerSize', 12, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Best Start');
plot(best_traj(end,1), best_traj(end,2), 'rp', 'MarkerSize', 15, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'Best End');

% Plot 5 additional trajectories
colors = lines(5);
for k = 1:5
    plot(extra_traj(:,1,k), extra_traj(:,2,k), '--', 'Color', colors(k,:), ...
         'LineWidth', 1, 'DisplayName', sprintf('Particle %d', k));
end

% Mark start and end
plot(trueTraj(1,1), trueTraj(1,2), 'gs', 'MarkerSize', 12, ...
     'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(trueTraj(end,1), trueTraj(end,2), 'gp', 'MarkerSize', 15, ...
     'MarkerFaceColor', 'g', 'DisplayName', 'End');

legend('Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Particle Filter Trajectories (M = 20)');
axis equal; grid on;
hold off;
