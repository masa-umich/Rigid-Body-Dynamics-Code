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

    newVb = y_trans*Vb;

    w = newVb(1);

    dz = newVb(3);
    newState = [x + dx, z + dz, u, w, q, stateTheta];
    
    % Calculate the inverse tangent of the vertical and horizontal velocities
    stateTheta = atan2(w, u);
    
    % Update the state vector with the new values
    newState = [x, z, u, w, q, stateTheta];

end



function [stateDerivs] = getNoseConeDerivs(state, percentage, mass, Iyy)
% Calculate state derivatives for nose cone using relative velocity equation

    % Extract states
    u = state(3);      % u - x-velocity in body frame (forward/right)
    w = state(4);      % w - z-velocity in body frame (up)
    q = state(5);      % q - angular velocity around y-axis
    theta = state(6);  % theta - pitch angle
    
    % Constants
    g = 9.81;
    rho = 1.225;
    CDp = 1.2;
    parA = 18.68; % parachute reference area [m²]
    
    % Apply deployment percentage
    effectiveParA = parA * percentage;

    % RELATIVE VELOCITY CALCULATION for parachute attachment point
    % v_B = v_A + ω × r_B/A
    % Coordinate system: x-right, z-up, y-into page
    
    % Point A: Center of Mass (where u,w are measured)
    v_A = [u; 0; w];  % Velocity at COM in body frame [u_x, 0, w_z]
    
    % Vector from COM to parachute attachment point
    % Parachute is typically at the front (nose) of the rocket
    dNoseconeGodplate = 1.92; % [m] - distance from nose tip to attachment
    dryCOM = 2.96;            % [m] - COM position from nose tip
    momentArm = dryCOM - dNoseconeGodplate; % Should be positive (COM behind parachute)
    
    % r_B_A: vector from COM (A) to parachute attachment (B)
    % Since parachute is at front, r_B_A points forward along x-axis
    r_B_A = [momentArm; 0; 0];  % [x, y, z] - positive x means forward
    
    % Angular velocity vector - rotation around y-axis
    omega = [0; q; 0];  % [ω_x, ω_y, ω_z] - only ω_y is non-zero
    
    % Velocity at parachute attachment point (point B)
    % v_B = v_A + ω × r_B/A
    v_B = v_A + cross(omega, r_B_A);
    
    % For cross product in our coordinate system:
    % ω × r = [ω_y*r_z - ω_z*r_y, ω_z*r_x - ω_x*r_z, ω_x*r_y - ω_y*r_x]
    % Since ω = [0, q, 0] and r = [momentArm, 0, 0]:
    % ω × r = [q*0 - 0*0, 0*momentArm - 0*0, 0*0 - q*momentArm] 
    %       = [0, 0, -q*momentArm]
    
    % So v_B = [u, 0, w - q*momentArm]
    
    % Transform body velocities to global frame for parachute forces
    % Rotation matrix for pitch around y-axis:
    R = [cos(theta),  0, sin(theta);
         0,           1, 0;
         -sin(theta), 0, cos(theta)];
    
    v_B_global = R * v_B;  % Velocity at parachute in global frame
    v_B_global_2D = [v_B_global(1); v_B_global(3)]; % Extract x,z components
    
    % Compute aerodynamic quantities
    vinf = norm(v_B_global_2D);
    
    if vinf < 1e-6
        vinf_unit_vec = [0; 0];
        alpha = 0;
    else
        vinf_unit_vec = v_B_global_2D ./ vinf;
        % Angle of attack - angle between velocity vector and body x-axis
        alpha = atan2(v_B_global_2D(2), v_B_global_2D(1));  
    end

    qinf = 0.5 * rho * vinf^2;                  

    % Parachute force (in global/wind frame) - opposes motion
    fParachute_global = -qinf * CDp * effectiveParA * vinf_unit_vec; % 2x1

    % Transform parachute force back to body frame
    % Inverse of rotation matrix R is R' for rotation matrices
    R_inv = [cos(theta), 0, -sin(theta);
             0,          1,  0;
             sin(theta), 0,  cos(theta)];
    
    fParachute_global_3D = [fParachute_global(1); 0; fParachute_global(2)];
    fParachute_body = R_inv * fParachute_global_3D;
    fParachute_body_2D = [fParachute_body(1); fParachute_body(3)];

    % Force components in body frame
    axialParaForce = fParachute_body_2D(1);  % Along body x-axis
    normalParaForce = fParachute_body_2D(2); % Along body z-axis

    % Moment calculation using cross product: M = r × F
    % r_B_A is from COM to parachute attachment
    % F is the force applied at the parachute attachment
    MVec_body = cross(r_B_A, fParachute_body);  % r × F
    Mchute = MVec_body(2);  % Y-component (pitching moment around y-axis)
    
    % EQUATIONS OF MOTION (in body frame)
    % Body frame accelerations
    du = -q * w + axialParaForce / mass - g * sin(theta);  % x-acceleration
    dw = q * u + normalParaForce / mass - g * cos(theta);  % z-acceleration  
    dq = Mchute / Iyy;      % angular acceleration around y-axis
    dtheta = q;             % angular rate
    
    % Position derivatives (transform body velocities to global frame)
    % Using rotation matrix to convert body velocities to global frame
    v_COM_global = R * [u; 0; w];  % COM velocity in global frame
    dx = v_COM_global(1);  % x-velocity in global frame
    dz = v_COM_global(3);  % z-velocity in global frame
    
    stateDerivs = [dx, dz, du, dw, dq, dtheta];
end




