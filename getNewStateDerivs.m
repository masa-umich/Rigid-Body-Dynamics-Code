function [newStateDerivs] = getNewStateDerivs(state, percentage, mass, Iyy,flightMode)

% Freefall after appogee
if flightMode == 1

    z = state(end,2); % - or + IDK
    u = state(end, 3);
    w = state(end, 4);
    q = state(end, 5);
    stateTheta = state(end, 6);
    psi = state(end, 7);
    
    CA = 0.1; %axial force coefficient NOT REAL!!!
    CN = -0.1; %normal force coefficient NOT REAL!!!
    CM = 0.1; %pitch moment coeficcient NOT REAL!!!
    
    %Getting Vinf and qinf
    vinf= getLimelightVinf(state, percentage); % (m/s)
    [~, ~, rho, ~] = atmosphere(z); % (kg/m^3)
    qinf = 0.5*rho*(vinf^2); % (Pa)
    
    g = 9.81; % gravity (m/s^2)
    rArea = 2.3680; % Limelight reference area (m^2)
    D = 7.4; %Limelight length (m)
    
    % Fixed coodinate system forces
    fAxial = -qinf*rArea*CA; % (N)
    fNormal = -qinf*rArea*CN; % (N)
    
    % Transform into body coordintes 
    fixedCoordsForceVec = [fAxial; 0; fNormal]; % (N)
    bodyCoordsForceVec = [cos(stateTheta) 0 sin(stateTheta);0 0 0;...
     -sin(stateTheta) 0 cos(stateTheta)]*fixedCoordsForceVec; % [N]
    
    % Forces in body coords
    fAxialBody = bodyCoordsForceVec(1);
    fNormalBody = bodyCoordsForceVec(3);
    
    % Pitch Moment
    My = CM * qinf * rArea * D;
    
    %Equations of Motion differentials
    
    du = (-q*w)+(fAxialBody/mass); %removed ru term as r is assumed 0
    dw = (q*u)+(fNormalBody/mass) - g; %removed -pv term as p and v is assumed 0
    dq = My / Iyy;
    dq = max(min(dq, 100), -100); % limit pitch acceleration
    dtheta = q*cos(psi); %removed r term
    if abs(cos(stateTheta)) < 1e-3
    dpsi = 0;
    else
    dpsi = q * sin(psi) * tan(stateTheta);
    end
    % Need psi in order to calculate theta
    dx = u * cos(stateTheta) - w * sin(stateTheta); 
    dz = u * sin(stateTheta) + w * cos(stateTheta);

    newStateDerivs = [dx, dz, du, dw, dq, dtheta, dpsi]; %COLUMN VECTOR FOR ODE45

%Pilot Chute Flight
elseif flightMode == 2

    z = state(end,2); % - or + IDK
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    stateTheta = state(end,6);
    psi = state(end,7);
    
    CA = 0.1; %axial force coefficient NOT REAL!!!
    CN = -0.2; %normal force coefficient NOT REAL!!!
    CM = 0.1; %pitch moment coeficcient NOT REAL!!!
    
    %Pilot Chute Specs
    cdPilot = 1.3;
    pilotArea = 0.56; %(m^2)

    %Getting Vinf and qinf
    vinf= getLimelightVinf(state, percentage); % (m/s)
    [~, ~, rho, ~] = atmosphere(z); % (kg/m^3)
    qinf = 0.5*rho*(vinf^2); % (Pa)
    
    g = 9.81; % gravity (m/s^2)
    rArea = 2.3680; % Limelight reference area (m^2)
    D = 7.4; %Limelight length (m)
    
    % Fixed coodinate system forces
    pilotDrag = 0.5*cdPilot*rho*(w^2)*pilotArea;
    pilotDrag = min(pilotDrag, 800);
    fAxial = qinf*rArea*CA - pilotDrag; % (N)
    fNormal = qinf*rArea*CN; % (N)
    
    % Transform into body coordintes 
    fixedCoordsForceVec = [fAxial; 0; fNormal]; % (N)
    bodyCoordsForceVec = [cos(stateTheta) 0 sin(stateTheta);0 0 0;...
     -sin(stateTheta) 0 cos(stateTheta)]*fixedCoordsForceVec; % [N]
    
    % Forces in body coords
    fAxialBody = bodyCoordsForceVec(1);
    fNormalBody = bodyCoordsForceVec(3);
    
    % Pitch Moment
    My = CM * qinf * rArea * D;
    
    %Equations of Motion differentials
    
    du = (-q*w)+(fAxialBody/mass); %removed ru term as r is assumed 0
    dw = (q*u)+(fNormalBody/mass) -g; %removed -pv term as p and v is assumed 0
    dq = My / Iyy;
    dq = max(min(dq, 100), -100); % limit pitch acceleration
    dtheta = q*cos(psi); %removed r term
    if abs(cos(stateTheta)) < 1e-3
    dpsi = 0;
    else
    dpsi = q * sin(psi) * tan(stateTheta);
    end
    % Need psi in order to calculate theta
    dx = u * cos(stateTheta) - w * sin(stateTheta); 
    dz = u * sin(stateTheta) + w * cos(stateTheta);
    
    newStateDerivs = [dx, dz, du, dw, dq, dtheta, dpsi]; %COLUMN VECTOR FOR ODE45
    
end


