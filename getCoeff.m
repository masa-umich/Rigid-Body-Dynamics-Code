function [CoD, CoL] = getCoeff(radA)
    % CoD = Coefficient of Drag
    % CoL = Coefficient of Lift
    % RA = Reference Area
    % AoA = Angle of Attack
<<<<<<< HEAD
    AoA = round(rad2deg(radA));
    M = readmatrix("C:\Users\hugoa\Rigid-Body-Dynamics-Code\FlipSimDMNEW.csv");
=======
    if (radA > pi)
        radA = 2*pi-radA;
    end
    AoA = round(rad2deg(abs(radA)));
    M = readmatrix("/Users/shreyasgorre/Documents/MATLAB/Rigid-Body-Dynamics-Code/FlipSimDMNEW.csv");
>>>>>>> 40072a56b9e57923c640f425c8a34ab46c2be34c
    D = M(:,3);
    L = M(:,4);
    A = M(:,7);
    CoD = interp1(A, D, AoA, 'linear');
    CoL = interp1(A, L, AoA, 'linear');
    if (radA > pi)
        CoD = -CoD;
        CoL = -CoL;
    end
    % RA = interp1(M(:,2), M(:,5), AoA, 'linear');
end
 

