function newState = getstate_btail(state)
% BODY-AXIS velocity at a boat-tail
% Returns [x z uB wB q theta]

    % Unpack
    x = state(end,1);
    z = state(end,2);
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    theta = state(end,6);

    % Geometry: base/boattail point relative to CG (BODY frame)
    dbtail = -6.0;              % m (axial station of base point)
    dryCOM = 3.0;               % m
    r_bt = [dryCOM - dbtail; 0; 0];

    % Rigid-body kinematics in BODY
    V_com = [u; 0; w];
    omega = [0; q; 0];
    V_bt  = V_com + cross(omega, r_bt);

    uB = V_bt(1);
    wB = V_bt(3);
    newState = [x, z, uB, wB, q, theta];
end