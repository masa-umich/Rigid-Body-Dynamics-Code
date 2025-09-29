function [vinf, vinf_vec, alpha] = getVinf(state, percentage)
%{
This function gets the total velocity vector magnitude and angle of attack
in respect to the vehicle based on the state vector that is given.
%}

    % get wind speed
    vw = getVw(state, percentage);

    u = state(4);
    w = state(5);
    theta = state(3);

    vinf = sqrt( (u - vw)^2 + (w)^2 );

    vinf_vec = [u-vw, w];

    vinfx = (u-vw)/cos(theta);
    vinfy = w/sin(theta);
     arg = (vinfx + vinfy) / vinf;
    arg = max(min(arg, 1), -1);   % clamp to [-1,1]
    alpha = acos(arg);
end

