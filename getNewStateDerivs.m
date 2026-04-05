function [newStateDerivs, fParachute] = getNewStateDerivs(state, percentage, mass, Iyy, ff, t_elapsed)

% Extract states
z = state(end,2);
u = state(end,3);
w = state(end,4);
q = state(end,5);
theta = state(end,6);

% Extract Parachute Extraction states
x_rel_d = state(end,7); % Drogue extraction distance
v_rel_d = state(end,8); % Drogue relative velocity
x_rel_m = state(end,9); % Main extraction distance
v_rel_m = state(end,10); % Main relative velocity


g = 9.81;
[~, ~, rho, ~] = atmosphere(z);
% Constants
CDp = 1.60; % pilot parachute
CDm = 1.60; % pilot parachute
parAp = 1.29; % parachute reference area [m^2] (pilot parachute)
parAd = 7.1; % drogue parachute
parAm = 177; % main parachute

%% Compute freestream
[qinf, vinf, vinf_vec, alpha] = getLimelightVinf(state, percentage);

vinf_vec = vinf_vec(:);                 % force column (2×1)
if norm(vinf_vec) < 1e-6
    vinf_unit_vec = [0;0];
else
    vinf_unit_vec = vinf_vec ./ norm(vinf_vec);   % keep 2×1
end

%fParachute = -qinf* CDp * parAp * vinf_unit_vec;

%% Compute Body Forces

mainDeployAlt = 609.6 + 305;

L_d = 7; % [m] Length of drogue tethers
mass_d = 0.013; % [kg] Mass of drogue parachute

L_main = 6.1; % [m] Length of main tethers
mass_p = 7.541; % [kg] Mass of main parachute

% Default derivatives for the extraction states (no movement)
dx_rel_m = 0; 
dv_rel_m = 0;
dx_rel_d = 0; 
dv_rel_d = 0;


%Parachute force (in fixed frame)
if z > (609.6 + 1524)
   fParachute = -qinf* CDp * parAp * vinf_unit_vec; % 2x1

elseif z > mainDeployAlt && x_rel_d < L_d
    fParachute = -qinf* CDp * parAp * vinf_unit_vec; % 2x1
    ff = true;

elseif z > mainDeployAlt && x_rel_d >= L_d
   ff = false;

   % if ((t_elapsed) < 3)
   %      CDm = CDp/3 * (t_elapsed);
   % end

   fParachute = -qinf* CDm * parAd * vinf_unit_vec; % 2x1

   dx_rel_d = 0;
   dv_rel_d = 0;

elseif z <= mainDeployAlt && x_rel_m < L_main
    fParachute = -qinf* CDp * parAd * vinf_unit_vec; % 2x1 drogue
    ff = true;

elseif x_rel_m >= L_main
    ff = false;
    % Lock the relative distance so it doesn't integrate to infinity
    dx_rel_m = 0;
    dv_rel_m = 0;

    % % Calculate Parachute force
    if ((t_elapsed) < 3)
        CDm = CDp/3 * (t_elapsed);
    end
    fParachute = -qinf* CDm * parAm * vinf_unit_vec; % 2x1
end

% Body to global
T = [ cos(theta)  -sin(theta);
     sin(theta)  cos(theta)];
% Fx is force in the x-direction (GLOBAL)
% Fz is the force in the z-direction (GLOBAL)
% Fx = 0;
% Fz = -g * mass;

%disp(Fx + " xz para + grav " + Fz);

[CoD, CoL] = getCoeff(alpha);

CoA = CoD*cos(theta) - CoL*sin(theta);

CoN = CoD*sin(theta) + CoL*cos(theta);

% RA is the reference Area of the rocket
RA = pi * 0.32385^2;
 
% FA is the axial force (body)
% FL is the normal force (body)

%disp(CoA + " aerial coeffs " + CoN);

FA = -0.5 * rho * CoA * RA * vinf^2;
FN = -0.5 * rho * CoN * RA * vinf^2;

%disp(FA + "body forces" + FN);

% NOTE: stateTheta is the angle that the rocket is angled
% NOT the angle of velocity. Since Drag and Lift are applied
% based on the angle of velocity, considering changing
% the following equations. -Hugo
% UPDATE: replaced stateTheta with velocityAngle
%disp("Lift:" + FL);
%disp("Drag:" + FD);

%% Utilize linear transformation
vec_x = [FA ; FN];
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
%CoM = 7.56 - (121.05 / 39.37); 
CoM = 3.048;
%disp("X: " + fParachute(1));
%disp("Z: " + fParachute(2));

% %Trying something here with projections
% Fn_unit = [cos(theta+pi/2);sin(theta+pi/2)];
% FNpara = norm(dot(fParachute,Fn_unit)*Fn_unit);

FNpara = fParachute(1) * sin(theta) + fParachute(2) * cos(theta);

disp("N: " + FNpara);
disp("V_inf: " + vinf_unit_vec);

% PD is the parachute distance from the tip of the nosecone
PD = 70 / 39.37;

%Technically negative torque, but FN is negative because of negative CoL
%from old coeff sheet
Torque = (CoP - CoM) * FN;
momentArm = CoM - PD;
body_vec = momentArm.*[cos(theta);sin(theta);0];

% Parachute forces after delay
if (ff == false)
    r = body_vec;
    F = fParachute(1:2);
    Torque = Torque + (r(1)*F(2) - r(2)*F(1));
    % if (theta > pi/2)
    %     theta;
    % end
    % if z < mainDeployAlt
    %     if (t_elapsed < 10)
    %         fParachute = fParachute.*((t_elapsed-2)/8); % 2x1
    %     end
    % end
    %display(fParachute(1));
    Fx = Fx + fParachute(1);
    Fz = Fz + fParachute(2);
end
% if (ff == true)
%     fParachute = 0;
% end

FA = Fx*cos(theta) + Fz*sin(theta);      % axial (along body x)
FN = -Fx*sin(theta) + Fz*cos(theta);     % normal (perp to body x)

% Need to make sure that du and dw are solved right (might
% need to add the Fa from parachute but not sure)
du = FA/mass + q*w;
dw = FN/mass - q*u;

% Update the angular acceleration to include damping
dq = (Torque) / Iyy;

% dx = u;
% dz = w;

dx =  u*cos(theta) - w*sin(theta);
dz =  u*sin(theta) + w*cos(theta);

% if dx < 0
%     dx;
% end

dtheta = q;

if z > mainDeployAlt && z < (609.6 + 1524) && x_rel_d < L_d
    
    % 1. Acceleration of the bag
    % Assuming fParachute here is calculated using the DROGUE area
    a_bag = norm(fParachute) / mass_d; 
    
    % 2. Acceleration of the rocket (along its flight path)
    % You already calculated du and dw. We approximate axial acceleration.
    a_rocket = du; 
    
    % 3. Relative kinematics
    dv_rel_d = a_bag - a_rocket; % Rate of change of relative velocity
    dx_rel_d = v_rel_d;            % Rate of change of relative distance

end

if z <= mainDeployAlt && x_rel_m < L_main
    
    % 1. Acceleration of the bag
    % Assuming fParachute here is calculated using the DROGUE area
    a_bag = norm(fParachute) / mass_p; 
    
    % 2. Acceleration of the rocket (along its flight path)
    % You already calculated du and dw. We approximate axial acceleration.
    a_rocket = du; 
    
    % 3. Relative kinematics
    dv_rel_m = a_bag - a_rocket; % Rate of change of relative velocity
    dx_rel_m = v_rel_m;            % Rate of change of relative distance

end

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

if (ff == true)
    fParachute = 0;
end

    
newStateDerivs = [dx, dz, du, dw, dq, dtheta, dx_rel_d, dv_rel_d, dx_rel_m, dv_rel_m]; %row vector
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