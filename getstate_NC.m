function [newState] = getstate_NC(state)
% Return new state for the nose cone

    % Extract states
    x = state(end,1);
    z = state(end,2);
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    stateTheta = state(end,6);
    r_nc = 0; %distance from point to nose cone
    g = 9.81;
    rho = 1.225;
    % Constants
    CDp = 1.2;
    parA = 18.68; % parachute reference area [m^2] (drogue parachute)
    
    
    Va = [u;0;w];

    Vb = Va + [0; 0; q * r_nc];
    
    y_trans = [cos(stateTheta), 0, sin(stateTheta); 0, 1, 0; -sin(stateTheta), 0, cos(stateTheta)];

    newVb = cross(Vb,y_trans);

    dx = newVb(1);

    dz = newVb(3);
    newState = [x + dx, z + dz, u, w, q, stateTheta];
    
    % Calculate the inverse tangent of the vertical and horizontal velocities
    stateTheta = atan2(w, u);
    
    % Update the state vector with the new values
    newState = [x, z, u, w, q, stateTheta];

end








