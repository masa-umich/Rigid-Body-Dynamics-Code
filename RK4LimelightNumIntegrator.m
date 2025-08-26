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
t =0 ;

% Wind for getVw
percentage = '50';

% Limelight Parameters
mass = 231; % vehicle dry mass (kg)
Iyy = 1750; % vehicle moment of inertia about the y-axis (kg*m^2) NOT REAL
rArea = 2.368; % (m^2)
D = 7.4; % (m)

% Inital state vector
state0 = [xp, zp, u, w, q, theta, psi];
state = state0;

%Time after appogee recovery bay is deployed
timeAfterAppogee = 0.5; %(s)


%Integrating for freefall

i = 1;
time(i,1) = t;

%freefall
while t <= timeAfterAppogee
    flightmode = 1;
    dt = 0.05;
    state(i+1,:) = RK4Solver(state(i,:),dt,percentage,mass,Iyy,flightmode);
    t = t+dt;
    i = i+1;
    time(i,1) = t;
end

%initial pilot chute deployment
while t < timeAfterAppogee + 2
    flightmode = 2;
    dt = 0.001;
    state(i+1,:) = RK4Solver(state(i,:),dt,percentage,mass,Iyy,flightmode);
    t = t +dt;
    i = i+1;
    time(i,1) = t;
end

%fall back down after parachute deployment
while state(i,2) < 0
    flightmode = 2;
    dt = 0.01;
    state(i+1,:) = RK4Solver(state(i,:),dt,percentage,mass,Iyy,flightmode);
    t = t +dt;
    i = i+1;
    time(i,1) = t;
end

plot(time,-state(:,2));
ylabel('Altitude (m)')
xlabel('Time (s)')
title('Altitude vs Time')