function newState = getstate_Fins(state)
% BODY velocity at a representative fin force point.
% Returns [x z uF wF q theta]

    x = state(end,1); 
    z = state(end,2);
    u = state(end,3);
    w = state(end,4);
    q = state(end,5);
    theta = state(end,6);

    % Geometry: fin force point relative to CG (BODY frame)
    dfin   = -5.0;              % m (axial station of fin point; negative if aft of CG reference)
    dryCOM = 3.0;               % m (axial CG location)
    r_f = [dryCOM - dfin; 0; 0];

    % Rigid-body kinematics in BODY
    V_com = [u; 0; w];
    omega = [0; q; 0];
    V_f   = V_com + cross(omega, r_f);

    uF = V_f(1);
    wF = V_f(3);
    newState = [x, z, uF, wF, q, theta];
end