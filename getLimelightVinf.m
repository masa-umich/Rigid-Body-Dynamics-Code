function [vinf, alpha] = getLimelightVinf(y, percentage)

% get wind speed
    vw = getVw(y, percentage);

    u = y(3);
    w = y(4);
    stateTheta = y(6);
    
% Divide by 0 protection

    smallVal = 1^(-6);
    cTheta = max(smallVal, cos(stateTheta));
    sTheta = max(smallVal, sin(stateTheta));

    vinf = sqrt( (u - vw)^2 + (w)^2 );

    maxVinf = max(vinf, smallVal);

    alpha = acos( min(1, max(-1, (((u - vw) / cTheta) ...
        + (w / sTheta)) / maxVinf)) );
end

