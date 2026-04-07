%% Monte Carlo

close all; clear; clc;

% simulation parameters
iterations = 200;
qSigma = 0.3/3; % according to shreyas
vSigma = 10/3; % according to shreyas
thetaSigma = (pi/3)/3; % according to shreyas

resultsMat = cell(iterations, 1);

% create a vector of normally distributed randomization factors
qRand = qSigma .* randn(iterations,1); % angular velocity
vRand = (vSigma .* randn(iterations,1)) + 20; % descent velocity
thetaRand = (thetaSigma .* randn(iterations,1)) + pi/2; % pitch angle

for i = 1:iterations

    rand = [qRand(i), vRand(i), thetaRand(i)];
    
    resultsMat{i} = mainSim(rand);

end

%% Present Results

max_forces = zeros(iterations,1);
max_angle = zeros(iterations,1);
for i = 1:iterations
    max_forces(i) = max(abs(resultsMat{i}(:,1)));
    max_angle(i) = max(abs(resultsMat{i}(:,2) - 90));
end

confidence_levels = [89, 94, 99];

load_bounds = prctile(max_forces, confidence_levels);
pitch_bounds = prctile(max_angle, confidence_levels);

clc; % clear command window

if isfile('Simulation_Output.txt')
    delete('Simulation_Output.txt');
end
% Start recording the command window
diary('Simulation_Output.txt');

fprintf('===== Shreyas and Hugo || Monte Carlo Simulation =====\n\n');

for i = 1:length(confidence_levels)

    prob = confidence_levels(i);
    peak_load = load_bounds(i);
    peak_pitch = pitch_bounds(i);
    
    fprintf('The max load remains below %.2f with a confidence of %d percent.\n', peak_load, prob);
    fprintf('The max pitch angle remains below %.4f deg with a confidence of %d percent.\n\n', peak_pitch, prob);
end

diary off;