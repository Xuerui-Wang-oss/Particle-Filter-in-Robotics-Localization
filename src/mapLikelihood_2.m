function w = mapLikelihood_2(x, z, Q, map, sensor_pos, n_rs_rays, max_range)

angles = linspace(27, -27, n_rs_rays)' * (pi / 180);

% Predict depth using the actual map
z_pred = predictDepth(x, map, angles, sensor_pos);

% Clamp z: NaN, non-positive, or beyond max_range -> max_range
z(isnan(z) | z <= 0 | z > max_range) = max_range;

% Clamp z_pred: NaN, Inf, non-positive, or beyond max_range -> max_range
z_pred(~isfinite(z_pred) | z_pred <= 0 | z_pred > max_range) = max_range;

innovation = z - z_pred;
n_v = length(z);
w = exp(-0.5 * innovation' * (Q \ innovation)) / ...
    sqrt((2*pi)^n_v * det(Q));

end
