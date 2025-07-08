function [value, isterminal, direction] = landingEvent(t, y)
    farAlt = -609.6;
    value = y(2) - farAlt; % Trigger when actual altitude crosses FAR
    isterminal = 1;
    direction = -1;
end