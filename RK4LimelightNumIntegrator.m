% state vector form: [xp,zp,u,w,q,theta,psi]

close all; clear; clc;

% FAR Launch Site Altutude
farAlt = -609.6; %(m)

%Appogee Input
appogee = -16741; %(m)

% Inputs
u = 50; % x component of COM velo. body coordinate system (m/s)
w = 0; % z component of COM velo. body coordinate system (m/s)
q = 0; % y component of rotation rate body coordinate syatem (rad/s)
theta = 0; % pitch (rad)
xp = 0; % x position in global coordinate system (m)
zp = appogee; % z position in global coordinate system (m)
t = 0 ;
rho = 1.225; % air density

% Wind for getVw
percentage = '50';

% Limelight Parameters
mass = 231; % vehicle dry mass (kg)
Iyy = 1750; % vehicle moment of inertia about the y-axis (kg*m^2) NOT REAL
rArea = 2.368; % (m^2)
D = 7.4; % (m)

% Inital state vector
state0 = [xp, zp, u, w, q, theta];
state = state0;

%Time after appogee recovery bay is deployed
timeAfterAppogee = 15; %(s)

%Integrating for freefall

i = 1;
time(i,1) = t;

%freefall
while state(end,2) < farAlt
    dt = 0.01;
    state(i+1,:) = RK4Solver(state(i,:),dt,percentage,mass,Iyy);
    t = t+dt;
    i = i+1;
    time(i,1) = t;
end

plot(time(1:end),-state(1:end,2));
ylabel('Altitude (m)')
xlabel('Time (s)')
title('Altitude vs Time')

disp(['Terminal velocity in pilot chute phase is: ',num2str(max(state(:,4))) , ' m/s'])
disp(['Limelight descended ',num2str(-1*(-state(end,2)+state(1,2))), ' m under pilot chute flight phase'])
