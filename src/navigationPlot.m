function navigationPlot(map, goal, k, lambda, points)
% NAVIGATIONPLOT Plot the navigation function over the sphere world.
%
%   NAVIGATIONPLOT(MAP, GOAL, K, LAMBDA, POINTS)
%
%   Uses the Rimon-Koditschek navigation function:
%       phi(q) = d_goal^k / (d_goal^k + lambda * beta(q))
%   where beta(q) is the product of all obstacle distances.
%
%   INPUTS:
%       map     - k_obs x 3 matrix [x_center y_center radius]
%       goal    - 1 x 2 vector [x_goal y_goal]
%       k       - tuning exponent
%       lambda  - scaling factor
%       points  - n x 2 matrix of [x y] points

nPts = size(points, 1);
phi_vals = zeros(nPts, 1);

for idx = 1:nPts
    q = points(idx, :);

    % Distance to goal
    d_goal = norm(q - goal);

    % Compute beta: product of obstacle distance functions
    beta = 1;
    for i = 1:size(map, 1)
        center = map(i, 1:2);
        radius = map(i, 3);
        dist_to_center = norm(q - center);

        if i == 1
            % Boundary: robot is inside
            b_i = radius^2 - dist_to_center^2;
        else
            % Obstacle: robot is outside
            b_i = dist_to_center^2 - radius^2;
        end
        b_i = max(b_i, 0);
        beta = beta * b_i;
    end

    % Navigation function
    phi_vals(idx) = d_goal^k / (d_goal^k + lambda * beta)^(1/k);
end

% Reshape for grid plotting
x_unique = unique(points(:,1));
y_unique = unique(points(:,2));
nx = length(x_unique);
ny = length(y_unique);

if nx * ny == nPts
    X = reshape(points(:,1), ny, nx);
    Y = reshape(points(:,2), ny, nx);
    Z = reshape(phi_vals, ny, nx);

    figure;
    surf(X, Y, Z, 'EdgeColor', 'none');
    colorbar;
    xlabel('X'); ylabel('Y'); zlabel('\phi');
    title('Navigation Function');
    view(3);

    figure;
    contour(X, Y, Z, 30);
    hold on;
    plot(goal(1), goal(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2);
    theta_circle = linspace(0, 2*pi, 100);
    for i = 1:size(map, 1)
        cx = map(i,1); cy = map(i,2); r = map(i,3);
        plot(cx + r*cos(theta_circle), cy + r*sin(theta_circle), 'k-', 'LineWidth', 2);
    end
    hold off;
    xlabel('X'); ylabel('Y');
    title('Navigation Function Contour');
    axis equal;
else
    figure;
    scatter3(points(:,1), points(:,2), phi_vals, 10, phi_vals, 'filled');
    colorbar;
    xlabel('X'); ylabel('Y'); zlabel('\phi');
    title('Navigation Function');
end

end
