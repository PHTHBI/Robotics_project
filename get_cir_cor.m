function [cir_cor, x04_vector] = get_cir_cor();
    %Define parameters
    R = 32;                           % Radius of the circle
    P_c = [150, 0, 120].';            % Center location of the circle in column vector form
    x04_straight = [150, 0, 0];       % Direction of x04 axis at phi = pi/2

    % Define the number of segments for the circle (360/10 = 36 segments in total)
    num_points = 36;                 
    theta = linspace(0, 2*pi, num_points + 1);  % Generate theta values from 0 to 2*pi

    % Calculate the circle points in 3D space
    circle_points = zeros(3, num_points + 1); % Initialize an empty matrix to store the points
    unit_vectors = zeros(3, num_points + 1);   % Initialize a matrix to store unit vectors

    for j = 1:(num_points + 1)
        % Calculate the current point on the circle
        circle_points(:, j) = P_c + R * [0; cos(theta(j)); sin(theta(j))];
    
        % Calculate phi for each point (same as theta in this context)
        phi = theta(j);
    
        % Calculate x04_circle for each point
        x04_circle = x04_straight + R * [0, cos(phi), 0];
    
        % Calculate the unit vector for the current point
        x04_circle_unit = x04_circle / norm(x04_circle);
    
        % Store the unit vector
        unit_vectors(:, j) = x04_circle_unit;
    end

    cir_cor = circle_points;
    x04_vector = unit_vectors;

end
