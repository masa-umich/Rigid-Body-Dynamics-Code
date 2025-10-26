function [Fn, Fa] = getNForce(AoA, q)

% Get coefficients from the getCoeff function using AoA
[Cd, Cl, Ar] = getCoeff(AoA);

%Calculate Normal Force
Fn = 0.5 * q * Cl * Ar;

% Calculate Axial Force
Fa = 0.5 * q * Cd * Ar;

end