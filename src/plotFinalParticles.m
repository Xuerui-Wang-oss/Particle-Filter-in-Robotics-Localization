% plotFinalParticles.m - Part (c): Plot final particles on the map
% Shows x, y positions only (no headings)

% Load saved data
pfData = load('pf_500.mat');
dataStore = pfData.dataStore;

% Load map
scriptDir = fileparts(mfilename('fullpath'));
mapData = load(fullfile(scriptDir, 'cornerMap.mat'));
fields = fieldnames(mapData);
map = mapData.(fields{1});

M = (size(dataStore.particles, 2) - 1) / 4;  % number of particles

% Extract final particles (last row)
rowEnd = dataStore.particles(end, :);
pData = rowEnd(2 : 3*M+1);
particles_final = reshape(pData, 3, M);  % 3-by-M

figure; hold on;

% Draw map walls directly from map matrix
for i = 1:size(map, 1)
    plot([map(i,1) map(i,3)], [map(i,2) map(i,4)], 'k-', 'LineWidth', 2, ...
         'HandleVisibility', 'off');
end

% Plot final particle positions
plot(particles_final(1, :), particles_final(2, :), 'r^', ...
     'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'r', ...
     'DisplayName', 'Final Particles');

% Plot true final position if available
if ~isempty(dataStore.truthPose)
    trueFinal = dataStore.truthPose(end, 2:3);
    plot(trueFinal(1), trueFinal(2), 'gp', 'MarkerSize', 15, ...
         'LineWidth', 2, 'MarkerFaceColor', 'g', 'DisplayName', 'True Position');
end

legend('Location', 'best');
xlabel('x [m]'); ylabel('y [m]');
title('Final Particle Set');
axis equal; grid on;
hold off;
