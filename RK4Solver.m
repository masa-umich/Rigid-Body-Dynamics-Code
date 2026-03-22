function [updatedState, fP_next, t_elapsed] = RK4Solver(state, dt, percentage, mass, Iyy, t, t_elapsed)
% RK4 numerical integration
% ff = are we in freefall? 
z = state(2);
ff = true;
if t > 1
    ff = false;
end
if z < (609.6 + 305)
    ff = true;
    t_elapsed = t_elapsed + dt;
    if t_elapsed > 2
        ff = false;
    end
end
    [f0, ~] = getNewStateDerivs(state, percentage, mass, Iyy, ff, t_elapsed);
    %disp(f0);
    [f1, ~] = getNewStateDerivs(state + 0.5*dt*f0, percentage, mass, Iyy, ff, t_elapsed);
    %disp(f1);
    [f2, ~] = getNewStateDerivs(state + 0.5*dt*f1, percentage, mass, Iyy, ff, t_elapsed);
    %disp(f2);
    [f3, ~] = getNewStateDerivs(state + dt*f2, percentage, mass, Iyy, ff, t_elapsed);
    %disp(f3);
    
    updatedState = state + (dt/6)*(f0 + 2*f1 + 2*f2 + f3);

    [~, fP_next] = getNewStateDerivs(updatedState, percentage, mass, Iyy, ff,  t_elapsed);

    %updatedState(6) = mod(updatedState(6), 2*pi);
    disp("state: " + updatedState);
    disp("t_elapsed: " + t_elapsed);
end



