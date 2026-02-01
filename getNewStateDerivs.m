function [newStateDerivs] = getNewStateDerivs(state, percentage, mass, Iyy, t)

% Extract states
z = state(end,2);
u = state(end,3);
w = state(end,4);
q = state(end,5);
theta = state(end,6);
g = 9.81;
[~, ~, rho, ~] = atmosphere(z);
% Constants
CDp = 0.7; % pilot parachute (estimation, 0.62-0.77)
parAp = 1.29; % parachute reference area [m^2] (pilot parachute)
parAd = 7.1; % drogue parachute
parAm = 177; % main parachute
% Below is the angle between the velocity vector and the x-axis

%% Compute freestream
[qinf, vinf, vinf_vec, alpha] = getLimelightVinf(state, percentage);

vinf_vec = vinf_vec(:);                 % force column (2×1)
if norm(vinf_vec) < 1e-6
    vinf_unit_vec = [0;0];
else
    vinf_unit_vec = vinf_vec ./ norm(vinf_vec);   % keep 2×1
end              

fParachute = -qinf* CDp * parAp * vinf_unit_vec;

%% Compute Body Forces

%Parachute force (in fixed frame)
% if z > (609.6 + 1524)
%    fParachute = -qinf* CDp * parAp * vinf_unit_vec; % 2x1
% elseif z > (609.6 + 305) 
%    fParachute = -qinf* CDp * parAd * vinf_unit_vec; % 2x1
% else 
%    fParachute = -qinf* CDp * parAm * vinf_unit_vec; % 2x1
% end

T = [ cos(theta)  sin(theta);
     -sin(theta)  cos(theta)];
% Fx is force in the x-direction (GLOBAL)
% Fz is the force in the z-direction (GLOBAL)
% Fx = 0;
% Fz = -g * mass;

%disp(Fx + " xz para + grav " + Fz);

[CoA, CoN] = getCoeff(theta);

% RA is the reference Area of the rocket
RA = pi * 0.32385^2;
 
% FA is the axial force (body)
% FL is the normal force (body)

%disp(CoA + " aerial coeffs " + CoN);

FA = -0.5 * rho * CoA * RA * vinf^2;
FN = 0.5 * rho * CoN * RA * vinf^2;

%disp(FA + "body forces" + FN);

% NOTE: stateTheta is the angle that the rocket is angled
% NOT the angle of velocity. Since Drag and Lift are applied
% based on the angle of velocity, considering changing
% the following equations. -Hugo
% UPDATE: replaced stateTheta with velocityAngle
%disp("Lift:" + FL);
%disp("Drag:" + FD);

%% Utilize linear transformation
vec_x = [FN ; FA];
forces = T * vec_x;

% Update global forces
Fx = 0;
Fz = -g * mass;

Fx = Fx + forces(1);
Fz = Fz + forces(2);

% Fx = Fx + FA * cos(theta) + FN * sin(theta);
% Fz = Fz + FA * sin(theta) + FN * cos(theta);

%disp(Fx + " xz final " + Fz);
%disp("X-Force:" + Fx);
%disp("Z-Force:" + Fz);

% Set the origin of the rocket body to the tip
% CoP = Center of Pressure distance from the tip of the nosecone
% CoM = Center of Mass distance from the tip of the nosecone
% alpha is the angle that the Torque due to drag
% and lift acts at based on the rocket's pitch and 
% direction of motion

CoP = 5.6;
CoM = 7.56 - (121.05 / 39.37); 
%disp("X: " + fParachute(1));
%disp("Z: " + fParachute(2));

FNpara = fParachute(1) * sin(theta) + fParachute(2) * cos(theta);

%disp("N: " + FNpara);

% PD is the parachute distance from the tip of the nosecone
PD = 70 / 39.37;

%Technically negative torque, but FN is negative because of negative CoL
%from old coeff sheet
Torque = (CoP - CoM) * FN;

% Parachute forces after delay
if (t >=2)
    if (theta < pi / 2)
        Torque = Torque + FNpara * PD;
    else 
        Torque = Torque - FNpara * PD;
    end
    %display(fParachute(1));
    %Fx = Fx + fParachute(1);
    Fz = Fz + fParachute(2);
end

FA = Fx*cos(theta) + Fz*sin(theta);      % axial (along body x)
FN = -Fx*sin(theta) + Fz*cos(theta);     % normal (perp to body x)

% Need to make sure that du and dw are solved right (might
% need to add the Fa from parachute but not sure)
du = FA/mass;
dw = FN/mass;
dq = Torque / Iyy; % Angular acceleration due to torque

% dx = u;
% dz = w;

dx =  u*cos(theta) - w*sin(theta);
dz =  u*sin(theta) + w*cos(theta);

dtheta = q;

%disp("Normal force: " + FN);

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
% 1. calculate aero forces in a fixed body frame 
% 2. calculate transformation matrix
% 3. transform aero forces into global coordinate system
% 4. add gravity to rotated aero forces
% 5. take components of new force vector in global coordinate system,
% divide by mass, to get your state derivatives in the global coordinate
% system
% 6. Find the normal forces on the NoseCone, Boattail, and Fins, then find
% the torque caused be each. Then take a Moment of Intertia from the master
% sheet and use to find angular acceleration (helps for finding theta)