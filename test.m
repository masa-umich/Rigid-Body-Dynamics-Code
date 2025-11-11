% --- test_getstate_NC.m ---
clear; clc;

% Example rocket state [x, z, u, w, q, theta]
% Units: meters, m/s, deg/s, degrees
state = [0, ...       % x-position (m)
          1000, ...   % z-position (m)
          50, ...     % u: axial velocity (m/s)
          -5, ...     % w: vertical velocity (m/s)
          deg2rad(5), ...      % q: pitch rate (deg/s)
          deg2rad(5)];         % theta: pitch angle (deg)

% Define constants
global rho Cn Ca
rho = 1.225;  % air density (kg/m^3)
Cn  = 0.8;    % normal force coefficient
Ca  = 0.5;    % axial force coefficient

% Add current folder to path
addpath(pwd);

% --- Run the function ---
newState = getstate_NC(state);

% --- Display results ---
disp('--- New Nose-Cone State ---');
fprintf('x      = %.2f m\n', newState(1));
fprintf('z      = %.2f m\n', newState(2));
fprintf('uN     = %.2f m/s\n', newState(3));
fprintf('wN     = %.2f m/s\n', newState(4));
fprintf('q      = %.3f deg/s\n', rad2deg(newState(5)));
fprintf('theta  = %.2f deg\n', rad2deg(newState(6)));