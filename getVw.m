function vw = getVw(y, percentage)
%{
This function returns the magnitude of velocity based on
the altitude of the vehicle and the wind percentage curve.
Converts altitude from meters to feet.

Inputs:
- state: A vector where state(2) represents altitude in meters.
- percentage: A string indicating the wind curve ('50', '90', '95', '99', '99.9').

Output:
- vw: Wind velocity in m/s.
%}

    alt = (-y(2) - 1219.9) * 3.28084; % Convert altitude from meters to feet.
    % wind data is geodetic, subtract ground elevation from altitude above
    % sea level.
    if ~isreal(y(2)) || isnan(y(2))
    error('Altitude (state(2)) is not real: %g + %gi', real(y(2)), imag(y(2)));
    end
    
    % Define altitude breakpoints (ft) and corresponding wind speeds (ft/s)
    switch string(percentage)
        case '0'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [0, 0, 0, 0, 0, 0, 0, 0, 0];
        case '50'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [12.1, 19.2, 24.5, 44.1, 65.0, 87.4, 67.6, 29.75, 29.75];
        case '60'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [14.55, 23.025, 31.65, 57.025, 83.05, 106.175, 80.65, 38.6875, 38.6875];
        case '70'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [17.0, 26.85, 38.8, 69.95, 101.1, 124.95, 93.7, 47.625, 47.625];
        case '80'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [19.45, 30.675, 45.95, 82.875, 119.15, 143.725, 106.75, 56.5625, 56.5625];
        case '90'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [21.9, 34.5, 53.1, 95.8, 137.2, 162.5, 119.8, 65.5, 65.5];
        case '95'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [25.5, 39.2, 61.4, 109.2, 156.0, 182.8, 133.2, 74.6, 74.6];

        case '99'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [32.3, 47.7, 76.2, 132.05, 188.4, 217.0, 155.1, 89.9, 89.9];

        case '99.9'
            altitudes = [0, 5000, 10000, 20000, 30000, 40000, 50000, 60000, 70000];
            speeds = [39.6, 57.2, 91.4, 154.5, 226.2, 250.0, 177.25, 105.5, 105.5];

        otherwise
            error('Invalid wind percentage');
    end
    
    % Linear interpolation for wind speed at given altitude
    vw_ft_s = interp1(altitudes, speeds, alt, 'linear', 'extrap');
    
    % Convert to m/s
    vw = vw_ft_s * 0.3048; 

end