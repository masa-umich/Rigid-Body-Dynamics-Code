% state vector form: [xp,zp,u,w,q,theta,psi]

close all; clear; clc;

% FAR Launch Site Altutude
farAlt = -609.6; %(m)

%Appogee Input
appogee = -16741; %(m)

% Inputs
u = 0; % x component of COM velo. body coordinate system (m/s)
w = 0; % z component of COM velo. body coordinate system (m/s)
q = 1; % y component of rotation rate body coordinate syatem (rad/s)
theta = pi/4; % picth (rad)
psi = pi/4;
xp = 0; % x position in global coordinate system (m)
zp = farAlt + appogee; % z position in global coordinate system (m)

% Wind for getVw
percentage = '50';

% Limelight Parameters
mass = 231; % vehicle dry mass (kg)
Iyy = 1750; % vehicle moment of inertia about the y-axis (kg*m^2) NOT REAL
rArea = 2.368; % (m^2)
D = 7.4; % (m)

% Inital state vector
state0 = [xp, zp, u, w, q, theta, psi];

%Time after appogee recovery bay is deployed
timeAfterAppogee = 0.5; %(s)

% For pilot chute flight
maxRuntime = 120; %(s)

%Integrating for freefall
flightMode = 1; %1 = freefall, 2 = pilot chute
t = [0,timeAfterAppogee]; %runtime for ODE45 for freefall
annonymousDerivsFunc = @(t,y) getNewStateDerivs(y, percentage, mass, Iyy,flightMode); ...
[tout1,stateout1] = ...
ode15s(annonymousDerivsFunc,t,state0); % Passes through deriv

%integrating for pilot parachute
flightMode = 2; %pilot chute
initialPilotState = stateout1(end, :); %last row in freefall state
t = [timeAfterAppogee,maxRuntime]; %runtime for ODE45 for pilot integration
terminate = odeset('Events', @landingEvent); %DOESNT WORK!!!!!!!!!!
annonymousDerivsFunc = @(t,y) getNewStateDerivs(y, percentage, mass, Iyy,flightMode); ...
[tout2,stateout2,landingTime,landingState,landingIndex] = ...
ode15s(annonymousDerivsFunc,t,initialPilotState,terminate); 

totaltout = [tout1; tout2];
totalstateout = [stateout1; stateout2];


%Plotting altitude vs. time
plot(totaltout,-totalstateout(:,2)); % z pos vs. time
title('Altitude vs. Time');
xlabel('Time (s)');
ylabel('Altitude (m)');
