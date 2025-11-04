function [stateDeriv] = getstate_Fins(state)
% Return new state for the nose cone

    % Extract states
    x = state(end,1);
    z = state(end,2);
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    stateTheta = state(end,6);
    r_fin = 0; %distance from point to nose cone
    g = 9.81;
    rho = 1.225;
    % Constants
    CDp = 1.2;
    parA = 18.68; % parachute reference area [m^2] (drogue parachute)

    % Fins

    % Fnum = 4; % # of fins
    % ls = 3; % fin span
    % lm = 2; %fin mid chord
    % lr = 3; %fin root chord
    % lt = 4; %fin tip chord
    dRT = 2; %diameter rocket tube
    % Cff = 2; %viscous friction coeff
    % Tf = 3; %fin thickness
    % dn = 2; %rocket diameter @ base of nosecone
    % Lts = 2; %total fin span 
    % Fin Normal Force (10 deg AOA MAX)
    % 
    % kfb = 1 + (dRT/2)/(ls+(drt/2));
    % CNFin = kfb * (4*Fnum*((ls/dn)^2))/(1+sqrt(1+(2*lm)/(lr+lt)));
    % 
    % Fin Drag Force NO AOA
    % 
    % Afp = (0.5*(lr+lt)*ls) + 0.5*dRT*lr; %fin platform area
    % CDf0 = 2*Cff*(1+2*(Tf/lm))*(4*Fnum*Afp)/(pi*(dRT^2));
    % 
    % Fin Interference Drag NO AOA
    % 
    % Afe = 0.5*(lr+lt)*ls;
    % CDfi0 = 2*Cff*(1+2*(Tf/lm))*(4*Fnum*(Afp-Afe))/(pi*(dRT)^2);
    % 
    % Fin Drag w/ AOA
    % Rs = Lts/dRT;
    % kbf = 0.1935*(Rs^2) + (0.8174*Rs)+1;
    % CDfalpha = (alpha^2)*((1.2*Afp*4)/(pi*(dRT^2))+3.12*(Kfb+kbf-1)*((4*Afe)/(pi*(dRT^2))));
    % 
    % CDFin = CDf0+CDfi0+CDfalpha;
    % 
    % 
    % Reference Area
    Sref = pi * (dRT/2)^2;

    V = sqrt(u^2+v^2);
    q_inf = 0.5 * rho * V^2;


    Fn_fin = q_inf * Sref * CN_fin;   % fin normal force
    Fa_fin = q_inf * Sref * CA_fin;   % fin axial force
    

    %[Fn, Fa] = getNForce(stateTheta, q); %Gives us the total Fn and Fa

    tau = Fn_fin*r_fin;

    dq = tau/I; % NEED TO ASSIGN I equal to the moment of intertia

    du = Fa_fin/m; % NEED TO ASSIGN m equal to the mass of the rocket

    dw = Fn_fin/m - g*sin(stateTheta); 

    % Update q using dq so that when calculating Vb, Fn is taken into
    % account.
    
    Va = [u;0;w];

    omega = [0; q; 0];       % pitch rate about y-axis
    r_vec = [-r_fin; 0; 0];

    Vb = Va + cross(omega,r_vec); %Update Vb

    
    y_trans = [cos(stateTheta), 0, sin(stateTheta); 0, 1, 0; -sin(stateTheta), 0, cos(stateTheta)]; %transformation about the y axis

    newVb = y_trans*Vb; %Using linear transformation

    dx = newVb(1);

    dz = newVb(3);
    
    dTheta = q;


    stateDeriv = [dx, dz, du, dw, dq, dTheta];

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