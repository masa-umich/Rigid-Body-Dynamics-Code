% state vector form: [xp,zp,u,w,q,theta]

close all; clear; clc;
format long g;
% FAR Launch Site Altutude
farAlt = 609.6; %(m)

% Apogee Input
apogee = 16741; %(m)

% Inputs
u = 50; % x component of COM velo. body (change to fixed) coordinate system (m/s)
w = 0; % z component of COM velo. body (change to fixed) coordinate system (m/s)
q = 0; % y component of rotation rate body coordinate syatem (rad/s)
theta = 0; % pitch (rad)
xp = 0; % x position in global coordinate system (m)
zp = apogee; % z position in global coordinate system (m)
t = 0 ;
x_rel = 0; 
v_rel = 0;
[~, ~, rho, ~] = atmosphere(zp);

% Wind for getVw
percentage = '0';

% Limelight Parameters
mass = 231; % vehicle dry mass (kg)
Iyy = 1750; % vehicle moment of inertia about the y-axis (kg*m^2) NOT REAL
rArea = 2.368; % (m^2)
D = 7.4; % (m)

% Inital state vector
state0 = [xp, zp, u, w, q, theta, x_rel, v_rel];
state = state0;

%Time after appogee recovery bay is deployed
timeAfterAppogee = 2; %(s)

%Integrating for freefall

i = 1;
time(i,1) = t;
t_elapsed = 0;

f_ch = [];   % parachute load history [N]
para_t = 0;
%freefall
while state(end,2) > farAlt
% while time(i) < 100
    dt = 0.1;
    if(state(end, 2) < 609.6 +305) 
        dt = 0.001;
    end
    if(t_elapsed > 2) 
        dt = 0.001;
    end
    if(t_elapsed > 5)
        dt = 0.1;
    end
    [state(i+1,:),fP, t_elapsed] = RK4Solver(state(i,:),dt,percentage,mass,Iyy,t, t_elapsed);
    f_ch(i+1) = norm(fP);
    t = t+dt;
    i = i+1;
    time(i,1) = t;
    
end

figure(1)
subplot(1,2,1)
plot(time,rad2deg(state(:,6)));
ylabel('Theta [deg]')
xlabel('Time (s)')
title('Theta [deg] i.r.t. horiz vs Time')

subplot(1,2,2)
plot(time,state(:,2));
ylabel('Altitude [m]')
xlabel('Time (s)')
title('Altitude vs Time')

figure(2)
plot(state(:,1), state(:,2))
ylabel('y pos [m]')
xlabel('x pos [m]')
title('Absolute position')

figure(3)
plot(time, f_ch)
ylabel('Force [N]')
xlabel('x pos [m]')
title('Force Graph')

%disp(['Terminal velocity in pilot chute phase is: ',num2str(min(state(:,4))) , ' m/s'])
disp(['Limelight descended ',num2str(1*(-state(end,2)+state(1,2))), ' m under pilot chute flight phase'])


[maxLoad, idx] = max(f_ch);

disp(['MAX parachute load = ', num2str(maxLoad), ' N'])
disp(['Occurs at t = ', num2str(time(idx)), ' s'])
disp(['Altitude at max load = ', num2str(state(idx,2)), ' m'])
