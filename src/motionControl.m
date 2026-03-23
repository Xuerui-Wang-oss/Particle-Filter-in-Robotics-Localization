function [dataStore] = motionControl(Robot, maxTime)
% Set default parameters
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 60;
end

try
    CreatePort = Robot.CreatePort;
catch
    CreatePort = Robot;
end

% Declare global variable
global dataStore;

% Initialize data structure
dataStore = struct('truthPose', [], ...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', [], ...
                   'deadReck', [], ...
                   'GPS', [], ...
                   'ekfMu', [], ...
                   'ekfSigma', [], ...
                   'particles', []);

%% ---- Load map and EKF parameters --------------------------------------
mapDir = fileparts(mfilename('fullpath'));  % directory of this .m file
mapData = load(fullfile(mapDir, 'cornerMap.mat'));
fields = fieldnames(mapData);
map = mapData.(fields{1});  % get the first (and likely only) variable

sensor_pos = [0, 0.08];    % sensor position in robot frame [x y]
n_rs_rays  = 9;             % number of depth rays
max_range  = 3;             % max sensor range [m]

%% ---- FILTER SELECTION ---------------------------------------------------
% Change this variable to switch between filters:
%   'EKF_GPS'   = EKF with GPS data
%   'EKF_depth' = EKF with depth data
%   'PF'        = Particle Filter with depth data
filterMethod = 'PF';

%% ---- Particle Filter parameters ----------------------------------------
M_particles = 500;   % number of particles (change to 500 for part (e))

% Noise covariances
R       = 0.01 * eye(3);                       % process noise (3x3)
Q_GPS   = 0.001 * eye(3);                      % GPS measurement noise (3x3)
Q_depth = 0.001 * eye(n_rs_rays);              % depth measurement noise (KxK)

%% ---- Initial conditions ------------------------------------------------
% Using true initial pose as the initial mu
mu_init    = [];
sigma_init = [2, 0, 0; 0, 2, 0; 0, 0, 0.1];

%% ---- Motion parameters (backupBump) ------------------------------------
wheel2Center = 0.13;
maxV         = 0.4;
fwdVel       = 0.2;
backupDist   = 0.1;
turnAngle    = 30 * pi / 180;

noRobotCount = 0;
state        = 1;       % 1=forward, 2=backup, 3=turn
accumDist    = 0;
accumAngle   = 0;

%% ---- Main loop ---------------------------------------------------------
SetFwdVelAngVelCreate(Robot, 0, 0);
tic

% First sensor read to get true initial pose
[noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

% Set initial pose: use mu_init if provided, otherwise use truthPose
if ~isempty(mu_init)
    mu = mu_init;                         % Q4(e): custom initial pose
else
    if ~isempty(dataStore.truthPose)
        mu = dataStore.truthPose(1, 2:4)';  % Q4(d): true initial pose
    else
        mu = [0; 0; 0];
    end
end

% Initialize EKF state
ekfMu    = mu;
ekfSigma = sigma_init;

% Initialize Particle Filter state
particles = [(-5 + 5 * rand(1, M_particles));      % x ~ U[-5, 0]
             (-5 + 10 * rand(1, M_particles));      % y ~ U[-5, 5]
             (-0.2 + 0.4 * rand(1, M_particles))];  % theta ~ U[-0.2, 0.2]

weights = ones(1, M_particles) / M_particles;        % uniform weights

% PF dynamics and likelihood functions
g_func = @(x, u) [x(1) + u(1) * cos(x(3) + u(2)/2);
                   x(2) + u(1) * sin(x(3) + u(2)/2);
                   x(3) + u(2)];
p_func = @(x, z, Q) mapLikelihood(x, z, Q, map, sensor_pos, n_rs_rays, max_range);

% Initialize deadReck, GPS, ekfMu, ekfSigma in dataStore
dataStore.deadReck  = [toc mu'];
dataStore.ekfMu     = [toc ekfMu'];
dataStore.ekfSigma  = ekfSigma;

% Store initial particles: each row = [timestamp, x1,y1,th1, x2,y2,th2, ...]
dataStore.particles = [toc, reshape(particles, 1, []), weights];

while toc < maxTime

    % Read and store sensor data
    [noRobotCount, dataStore] = readStoreSensorData(Robot, noRobotCount, dataStore);

    %% -- Get bump sensor --
    if ~isempty(dataStore.bump)
        latestBump = dataStore.bump(end, :);
        bumped = latestBump(2) || latestBump(3) || latestBump(7);
    else
        bumped = false;
    end

    %% -- Get odometry increments --
    if size(dataStore.odometry, 1) >= 1
        deltaD = dataStore.odometry(end, 2);
        deltaA = dataStore.odometry(end, 3);
    else
        deltaD = 0;
        deltaA = 0;
    end

    %% -- Dead reckoning (integrateOdom) --
    if isempty(dataStore.deadReck)
        % First step: use truthPose if available, else [0;0;0]
        if ~isempty(dataStore.truthPose)
            initPose = dataStore.truthPose(1, 2:4)';
        else
            initPose = [0; 0; 0];
        end
        drPose = integrateOdom(initPose, deltaD, deltaA);
    else
        prevPose = dataStore.deadReck(end, 2:4)';
        drPose = integrateOdom(prevPose, deltaD, deltaA);
    end
    dataStore.deadReck = [dataStore.deadReck; toc drPose(:, end)'];

    %% -- Run EKF --
    ut = [deltaD; deltaA];

    % Noisy GPS measurement: truthPose + Gaussian noise ~ N(0, Q_GPS)
    if ~isempty(dataStore.truthPose)
        truePose = dataStore.truthPose(end, 2:4)';
        z_gps = truePose + chol(Q_GPS)' * randn(3, 1);
        dataStore.GPS = [dataStore.GPS; toc z_gps'];
    else
        z_gps = ekfMu;  % no measurement available, use current estimate
    end

    % Depth measurement
    if ~isempty(dataStore.rsdepth)
        % Column 1 = timestamp, column 2 has issues, depth rays start at column 3
        z_depth = dataStore.rsdepth(end, 3:(n_rs_rays + 2))';
        % Invalid depth values (0 or negative) set to NaN
        z_depth(z_depth <= 0) = NaN;
        % Apply max_range: readings beyond max_range are treated as "no wall"
        z_depth(z_depth > max_range) = NaN;
    else
        z_depth = nan(n_rs_rays, 1);
    end

    %% -- Filter update (EKF or PF) --
    if ~isempty(dataStore.truthPose)
        truePose = dataStore.truthPose(end, 2:4)';
        angles_dbg = linspace(27, -27, n_rs_rays)' * (pi / 180);
        z_pred_true = predictDepth(truePose, map, angles_dbg, sensor_pos);
        fprintf('z_depth:     '); fprintf('%.3f ', z_depth); fprintf('\n');
        fprintf('z_pred_true: '); fprintf('%.3f ', z_pred_true); fprintf('\n\n');
    end
    switch filterMethod
        case 'EKF_GPS'
            [ekfMu, ekfSigma, ~, ~] = ...
                testEKF(ekfMu, ut, ekfSigma, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays);

        case 'EKF_depth'
            [~, ~, ekfMu, ekfSigma] = ...
                testEKF(ekfMu, ut, ekfSigma, R, z_gps, Q_GPS, z_depth, Q_depth, map, sensor_pos, n_rs_rays);

        case 'PF'
            [particles, weights,particles_pre,weights_pre] = PF(particles, weights, ut, z_depth, g_func, p_func, R, Q_depth);
    end

    % Store EKF estimates
    dataStore.ekfMu    = [dataStore.ekfMu; toc ekfMu'];
    dataStore.ekfSigma = [dataStore.ekfSigma; ekfSigma];

    %% --- debug ---
    true_pose = dataStore.truthPose(end, 2:4)';
    [~, best_idx] = max(weights_pre);
    best_pose = particles_pre(:, best_idx);

    % Calculate true position and the best particle weight
    w_true  = mapLikelihood(true_pose, z_depth, Q_depth, map, sensor_pos, n_rs_rays, max_range);
    w_best  = mapLikelihood(best_pose, z_depth, Q_depth, map, sensor_pos, n_rs_rays, max_range);

    fprintf('t=%.1f | w_true=%.4e | w_best=%.4e | best_pos=(%.2f,%.2f) | true_pos=(%.2f,%.2f)\n', ...
        toc, w_true, w_best, best_pose(1), best_pose(2), true_pose(1), true_pose(2));

    % Store PF particles: [timestamp, x1,y1,th1,..., w1,w2,...]
    dataStore.particles = [dataStore.particles; toc, reshape(particles_pre, 1, []), weights_pre];

    %% -- State machine control (backupBump) --
    switch state
        case 1  % Forward
            if bumped
                state = 2;
                accumDist = 0;
                cmdV = 0; cmdW = 0;
            else
                cmdV = fwdVel;
                cmdW = 0;
            end

        case 2  % Backup
            accumDist = accumDist + abs(deltaD);
            if accumDist >= backupDist
                state = 3;
                accumAngle = 0;
                cmdV = 0; cmdW = 0;
            else
                cmdV = -fwdVel;
                cmdW = 0;
            end

        case 3  % Turn clockwise
            accumAngle = accumAngle + abs(deltaA);
            if accumAngle >= turnAngle
                state = 1;
                cmdV = 0; cmdW = 0;
            else
                cmdV = 0;
                cmdW = -0.5;
            end
    end

    % Limit velocity commands
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxV, wheel2Center);

    % Send commands
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0, 0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW);
    end

    pause(0.1);
end

% Stop robot
SetFwdVelAngVelCreate(Robot, 0, 0);

end
