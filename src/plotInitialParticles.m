% plotInitialParticles.m - Part (b): Plot initial particles on the map
% Shows x, y positions and headings for each particle

% Load saved data
pfData = load('pf_500.mat');
dataStore = pfData.dataStore;

% Load map
scriptDir = fileparts(mfilename('fullpath'));
mapData = load(fullfile(scriptDir, 'cornerMap.mat'));
fields = fieldnames(mapData);
map = mapData.(fields{1});

M = (size(dataStore.particles, 2) - 1) / 4;  % number of particles

% Extract initial particles (first row)
row1 = dataStore.particles(1, :);
pData = row1(2 : 3*M+1);
particles_init = reshape(pData, 3, M);  % 3-by-M

arrowLen = 0.3;

figure; hold on;

% Draw map walls directly from map matrix
for i = 1:size(map, 1)
    plot([map(i,1) map(i,3)], [map(i,2) map(i,4)], 'k-', 'LineWidth', 2, ...
         'HandleVisibility', 'off');
end

% Plot each particle with heading arrow
for i = 1:M
    px = particles_init(1, i);
    py = particles_init(2, i);
    pth = particles_init(3, i);

    plot(px, py, 'bo', 'MarkerSize', 6, 'LineWidth', 1.5);
    quiver(px, py, arrowLen*cos(pth), arrowLen*sin(pth), 0, ...
           'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
end

xlabel('x [m]'); ylabel('y [m]');
title('Initial Particle Set with Headings');
axis equal; grid on;
hold off;
