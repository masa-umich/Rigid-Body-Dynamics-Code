function [qinf, alpha] = getBoattailVinf(ub, wb, z, percentage)
%{
This function gets the total velocity vector magnitude and angle of attack
in respect to the vehicle based on the state vector that is given.
%}
    rho = 1.225;
    % get wind speed
    vw = getVw(z, percentage);
    u = ub;
    w = wb;

    u_rel = u - vw;
    w_rel = w;
    
    vinf = hypot(u_rel, w_rel);
    vinf_vec = [u-vw, w];

    qinf = 0.5 * vinf^2 * rho;S
   
    alpha = atan2(w_rel, u_rel);
end