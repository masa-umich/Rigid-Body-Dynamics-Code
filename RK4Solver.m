function [updatedState] = RK4Solver(state,dt, percentage, mass, Iyy, flightmode)

% RK4 numerical integration
    f0 = getNewStateDerivs(state, percentage, mass, Iyy,flightmode);
    f1 = getNewStateDerivs(state + 0.5*dt*f0, percentage, mass, Iyy, flightmode);
    f2 = getNewStateDerivs(state + 0.5*dt*f1, percentage, mass, Iyy, flightmode);
    f3 = getNewStateDerivs(state + dt*f2, percentage, mass, Iyy, flightmode);
    
    updatedState = state + (dt/6)*(f0 + 2*f1 + 2*f2 + f3);
end