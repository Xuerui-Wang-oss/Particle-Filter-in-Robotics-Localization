function [particles, weights, particle_pre, weights_pre] = PF(particles, weights, ut, z, g_func, p_func, R, Q)

M = size(particles, 2);  % number of particles

%% ---- Prediction Step ----------------------------------------------------
% Propagate each particle through the dynamics model with added noise
noise = chol(R)' * randn(3, M);  % 3-by-M noise samples
for i = 1:M
    particles(:, i) = g_func(particles(:, i), ut) + noise(:, i);
end

%% ---- Update Step --------------------------------------------------------
% Compute weight for each particle based on measurement likelihood
for i = 1:M
    weights(i) = weights(i) * p_func(particles(:, i), z, Q);
end

% Normalize weights
w_sum = sum(weights);
if w_sum > 0
    weights = weights / w_sum;
else
    % If all weights are zero, reset to uniform
    weights = ones(1, M) / M;
end

%% ----Store weights before resampling -----
particle_pre = particles;
weights_pre = weights;

%% ---- Resampling Step (low-variance resampling) --------------------------
new_particles = zeros(size(particles));
r = rand() / M;
c = weights(1);
j = 1;

for i = 1:M
    u = r + (i - 1) / M;
    while u > c
        j = j + 1;
        c = c + weights(j);
    end
    new_particles(:, i) = particles(:, j);
end

particles = new_particles;
weights   = ones(1, M) / M;  % reset weights after resampling

end
