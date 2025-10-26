function [CoD, CoL, RA] = getCoeff(AoA)
    % CoD = Coefficient of Drag
    % CoL = Coefficient of Lift
    % RA = Reference Area
    % AoA = Angle of Attack
    M = readmatrix("Rigid-Body-Dynamics-Code\Kavin_CFD_Values.csv");
    CoD = interp1(M(:,2), M(:,3), AoA, 'linear');
    CoL = interp1(M(:,2), M(:,4), AoA, 'linear');
    RA = interp1(M(:,2), M(:,5), AoA, 'linear');
end
 

