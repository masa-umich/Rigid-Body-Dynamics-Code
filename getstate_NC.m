function newState = getstate_NC(state)
% BODY velocity at a nose-cone
% Returns [x z uN wN q theta]

    x = state(end,1);
    z = state(end,2);
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    theta = state(end,6);

    % Geometry: vector from CG to nose point (BODY frame)
    dNosecone = 2.0;            % m (location of nose point)
    dryCOM    = 3.0;            % m (CG location rocket)
    r_nc = [dryCOM - dNosecone; 0; 0];   % [rx; ry; rz]

    % Rigid-body kinematics in BODY
    V_com = [u; 0; w];
    omega = [0; q; 0];
    V_nc  = V_com + cross(omega, r_nc);

    uN = V_nc(1);
    wN = V_nc(3);
    newState = [x, z, uN, wN, q, theta];
end