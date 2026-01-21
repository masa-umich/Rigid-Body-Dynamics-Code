function [newStateDerivs] = getNewStateDerivs(state, percentage, mass, Iyy)

% Extract states
z = state(end,2);
u = state(end,3);
w = state(end,4);
q = state(end,5);
stateTheta = state(end,6);
g = 9.81;
[t, p, rho, a] = atmosphere(z);
% Constants
CDp = 1.2;
parA = 18.68; % parachute reference area [m^2] (drogue parachute)
% Below is the angle between the velocity vector and the x-axis
velocityAngle = atan2(w, u);

% Compute freestream
[vinf, vinf_vec, qinf, al] = getBoattailVinf(u, w, z, percentage);

vinf_vec = vinf_vec(:);                 % force column (2×1)
if norm(vinf_vec) < 1e-6
    vinf_unit_vec = [0;0];
else
    vinf_unit_vec = vinf_vec ./ norm(vinf_vec);   % keep 2×1
end              

% Parachute force (in fixed frame)

fParachute = -qinf* CDp * parA * vinf_unit_vec; % 2x1

T = [ cos(stateTheta)  sin(stateTheta);
     -sin(stateTheta)  cos(stateTheta)];
% Fx is force in the x-direction (GLOBAL)
% Fz is the force in the z-direction (GLOBAL)
Fx = fParachute(1);
Fz = fParachute(2) - g;

[CoD, CoL] = getCoeff(stateTheta);

% RA is the reference Area of the rocket
RA = pi * 0.32385^2;

% FD is the drag force (body)
% FL is the lift force (body)
FD = -0.5 * rho * CoD * RA;
FL = -0.5 * rho * CoL * RA;

% NOTE: stateTheta is the angle that the rocket is angled
% NOT the angle of velocity. Since Drag and Lift are applied
% based on the angle of velocity, considering changing
% the following equations. -Hugo
% UPDATE: replaced stateTheta with velocityAngle

Fx = Fx + FD * cos(velocityAngle) + FL * sin(velocityAngle);
Fz = Fz + FD * sin(velocityAngle) + FL * cos(velocityAngle);

du = Fx/mass;
dw = Fz/mass;

% Set the origin of the rocket body to the tip
% CoP = Center of Pressure
% CoM = Center of Mass
% alpha is the angle that the Torque due to drag
% and lift acts at based on the rocket's pitch and 
% direction of motion

CoP = 5.6;
CoM = 7.56 - (121.05 / 39.37); 

alpha = stateTheta - velocityAngle;

Torque = (CoP - CoM) * (FL * cos(alpha) + FD * sin(alpha));

dq = Torque / Iyy; % Angular acceleration due to torque

dx = u;
dz = w;
dtheta = q;

%Transform the force vector from the GLOBAL frame to the BODY frame.
% fVec_body_2D = T * fParachute;

%Unit vectors direction for axial and normal force
% u_axial = [cos(alpha); -sin(alpha)];  % Forward
% u_normal = [sin(alpha); cos(alpha)]; % Downward

%Axial and Normal Force Vecs
% axialParaForce = dot(fVec_body_2D, u_axial);
% normalParaForce = dot(fVec_body_2D, u_normal);

% Moment arm from COM to parachute
% dNoseconeGodplate = 1.92; % [m]
% dryCOM = 2.96;            % [m]
% momentArm = dryCOM - dNoseconeGodplate;
% rVec_body = [-momentArm; 0; 0];                          % 2×1
% fVec_body = [fVec_body_2D(1); 0; fVec_body_2D(2)];       % 2×1
% MVec_body = cross(rVec_body, fVec_body);
% Mchute    = MVec_body(2);
    
%Equations of Motion differentials
% du = -q*w + axialParaForce/mass +g*sin(stateTheta);
% dw =  q*u + normalParaForce/mass+g*cos(stateTheta);
% dq = Mchute / Iyy;
% dtheta = q;

% dx = u * cos(stateTheta) - w * sin(stateTheta); 
% dz = u * sin(stateTheta) + w * cos(stateTheta);

    
newStateDerivs = [dx, dz, du, dw, dq, dtheta]; %row vector
end

% Fins

%Fnum = 4; % # of fins
%ls = 3; % fin span
%lm = 2; %fin mid chord
%lr = 3; %fin root chord
%lt = 4; %fin tip chord
%dRT = 2; %diameter rocket tube
%Cff = 2; %viscous friction coeff
%Tf = 3; %fin thickness
%dn = 2; %rocket diameter @ base of nosecone
%Lts = 2; %total fin span 
% Fin Normal Force (10 deg AOA MAX)

%kfb = 1 + (dRT/2)/(ls+(drt/2));
%CNFin = kfb * (4*Fnum*((ls/dn)^2))/(1+sqrt(1+(2*lm)/(lr+lt)));

% Fin Drag Force NO AOA

%Afp = (0.5*(lr+lt)*ls) + 0.5*dRT*lr; %fin platform area
%CDf0 = 2*Cff*(1+2*(Tf/lm))*(4*Fnum*Afp)/(pi*(dRT^2));

%Fin Interference Drag NO AOA

%Afe = 0.5*(lr+lt)*ls;
%CDfi0 = 2*Cff*(1+2*(Tf/lm))*(4*Fnum*(Afp-Afe))/(pi*(dRT)^2);

% Fin Drag w/ AOA
%Rs = Lts/dRT;
%kbf = 0.1935*(Rs^2) + (0.8174*Rs)+1;
%CDfalpha = (alpha^2)*((1.2*Afp*4)/(pi*(dRT^2))+3.12*(Kfb+kbf-1)*((4*Afe)/(pi*(dRT^2))));

%CDFin = CDf0+CDfi0+CDfalpha;

%% Caspar Workflow:
% 1. calculate forces in a fixed body frame
% 2. calculate transformation matrix
% 3. transform forces into global coordinate system
% 4. take components of new force vector in global coordinate system,
% divide by mass, to get your state derivatives in the global coordinate
% system
% 5. Find the normal forces on the NoseCone, Boattail, and Fins, then find
% the torque caused be each. Then take a Moment of Intertia from the master
% sheet and use to find angular acceleration (helps for finding theta)