function [vinf, vinf_vec, alpha] = getNcVinf(uN, wN, z, percentage)
%{
This function gets the total velocity vector magnitude and angle of attack
in respect to the vehicle based on the state vector that is given.
%}

    % get wind speed
    vw = getVw(z, percentage);
    

    u_rel = uN - vw;
    w_rel = wN;

    vinf = hypot(u_rel, w_rel);
    vinf_vec = [uN-vw, wN];
   
    alpha = atan2(w_rel, u_rel);
end