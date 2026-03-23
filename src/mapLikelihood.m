function w = mapLikelihood(x, z, Q, map, sensor_pos, n_rs_rays, max_range)

angles = linspace(27, -27, n_rs_rays)' * (pi / 180);

% Predict depth using the actual map
z_pred = predictDepth(x, map, angles, sensor_pos);

% Valid sensor rays: finite, positive, within max_range
valid = ~isnan(z) & z > 0 & z <= max_range;

if ~any(valid)
    % No valid sensor rays (all NaN/max_range) -> uninformative, uniform weight
    w = 1;
    return;
end

% Only use rays where sensor has valid readings
z_v = z(valid);
z_pred_v = z_pred(valid);
Q_v = Q(valid, valid);

% For valid sensor rays where z_pred is NaN/Inf/invalid, set z_pred to max_range
% (sensor sees a wall but particle doesn't -> penalize)
z_pred_v(~isfinite(z_pred_v) | z_pred_v <= 0 | z_pred_v > max_range) = max_range;

innovation = z_v - z_pred_v;
n_v = length(z_v);
w = exp(-0.5 * innovation' * (Q_v \ innovation)) / ...
    sqrt((2*pi)^n_v * det(Q_v));

end
