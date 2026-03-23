function w = depthLikelihood(x, z, Q)
K = length(z);
angles = linspace(27, -27, K)' * (pi / 180);  % sensor angles in robot frame

z_pred = zeros(K, 1);
for k = 1:K
    global_angle = x(3) + angles(k);
    sin_angle = sin(global_angle);
    if sin_angle > 1e-6
        % Distance to wall at y=1 along this ray
        range = (1 - x(2)) / sin_angle;
        if range > 0
            % Depth = range projected onto sensor axis (cos of sensor angle)
            z_pred(k) = range * cos(angles(k));
        else
            z_pred(k) = NaN;
        end
    else
        z_pred(k) = NaN;  % ray doesn't hit wall
    end
end

% Only use valid measurements
valid = ~isnan(z) & ~isnan(z_pred);

if any(valid)
    innovation = z(valid) - z_pred(valid);
    Q_v = Q(valid, valid);
    % Multivariate Gaussian likelihood
    n_v = sum(valid);
    w = exp(-0.5 * innovation' * (Q_v \ innovation)) / ...
        sqrt((2*pi)^n_v * det(Q_v));
else
    w = 1e-300;  % no valid measurements, particle is invalid
    fprintf('Warning: no valid measurements for particle at [%.2f, %.2f, %.2f]\n', x(1), x(2), x(3));
end

end
