function depth = predictDepth(x, map, angles, sensor_pos)
% predictDepth - Predict depth measurements for a robot pose
%
% Given the robot pose and a closed map, ray-cast in each sensor direction
% and return the depth to the nearest wall. In a closed map every ray is
% guaranteed to hit a wall, so the output contains no NaN.
%
% Inputs:
%   x          - robot pose [x_pos; y_pos; theta]  (3x1)
%   map        - Mx4 wall matrix [x1, y1, x2, y2]  (same format as cornerMap)
%   angles     - sensor angles in the ROBOT frame   (Kx1 or 1xK), [rad]
%   sensor_pos - sensor position in the robot frame [x y]  (1x2)
%
% Output:
%   depth - predicted depth measurements  (Kx1)

theta = x(3);

angles = angles(:);   % ensure column vector
K      = length(angles);
depth  = nan(K, 1);

% Transform sensor from body frame to global frame
px = x(1) + sensor_pos(1)*cos(theta) - sensor_pos(2)*sin(theta);
py = x(2) + sensor_pos(1)*sin(theta) + sensor_pos(2)*cos(theta);

for k = 1:K
    % Convert sensor angle (robot frame) to global frame
    alpha = theta + angles(k);
    dx    = cos(alpha);
    dy    = sin(alpha);

    min_t = inf;

    for i = 1:size(map, 1)
        x1 = map(i,1);  y1 = map(i,2);
        x2 = map(i,3);  y2 = map(i,4);

        wx = x2 - x1;
        wy = y2 - y1;

        % Solve  [dx, -wx;  dy, -wy] * [t; s] = [x1-px; y1-py]
        det_A = dx * (-wy) - (-wx) * dy;   % = wx*dy - dx*wy

        if abs(det_A) < 1e-10
            continue;   % ray is parallel to wall segment
        end

        bx = x1 - px;
        by = y1 - py;

        % Cramer's rule
        t = (bx * (-wy) - (-wx) * by) / det_A;
        s = (dx  *   by -   dy  * bx) / det_A;

        % Valid intersection: ray goes forward (t>0), on segment (s in [0,1])
        if t > 1e-9 && s >= -1e-9 && s <= 1 + 1e-9
            if t < min_t
                min_t = t;
            end
        end
    end

    depth(k) = min_t * cos(angles(k));
end

end
