map = load('hw5Sphere.txt');
figure; hold on; axis equal;
theta = linspace(0, 2*pi, 100);
for i = 1:size(map,1)
    plot(map(i,1) + map(i,3)*cos(theta), map(i,2) + map(i,3)*sin(theta), 'k-', 'LineWidth', 2);
end
title('Sphere World');