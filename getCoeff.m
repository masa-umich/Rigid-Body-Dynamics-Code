function [CoD, CoL] = getCoeff(AoA)
    % CoD = Coefficient of Drag
    % CoL = Coefficient of Lift
    % RA = Reference Area
    % AoA = Angle of Attack
    M = readmatrix("/Users/shreyasgorre/Documents/MATLAB/Rigid-Body-Dynamics-Code/FlipSimDMNEW.csv");
    D = M(:,3);
    L = M(:,4);
    A = M(:,7);
    CoD = interp1(A, D, AoA, 'linear');
    CoL = interp1(A, L, AoA, 'linear');
    % RA = interp1(M(:,2), M(:,5), AoA, 'linear');
end