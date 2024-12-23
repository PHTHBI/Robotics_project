function crosses = find_crosses(image)
    % FIND_CROSSES detects the centers of crosses (X shapes) in the given image.
    % Input:
    %   - image: The input image (RGB or grayscale).
    % Output:
    %   - crosses: Nx2 array of [x, y] coordinates for detected crosses.

    % Convert to grayscale if necessary
    if size(image, 3) == 3
        image = rgb2gray(image);
    end

    % Enhance contrast to make edges more pronounced
    image = imadjust(image);  % Contrast enhancement

    % Smooth the image to reduce noise
    smoothed_image = imgaussfilt(image, 1.8);  % Slightly less smoothing for better detail retention

    % Apply edge detection using the Canny method with increased sensitivity
    edges = edge(smoothed_image, 'Canny', [0.05, 0.3]); % Increased sensitivity

    % Perform Hough Transform to detect lines
    [H, theta, rho] = hough(edges);

    % Find peaks in the Hough Transform with a lower threshold
    peaks = houghpeaks(H, 40, 'Threshold', ceil(0.25 * max(H(:)))); % More sensitive to weaker lines

    % Get lines based on Hough Transform peaks
    lines = houghlines(edges, theta, rho, peaks);

    % Initialize variables to store processed diagonal lines
    diagonal_lines = [];
    min_length = 25;  % Reduced minimum line length to detect smaller features
    angle_tolerance = 17;  % Broadened angle tolerance for increased sensitivity
    min_endpoint_separation = 15;  % Minimum distance between endpoints for diagonal lines

    % Analyze the detected lines
    for k = 1:length(lines)
        % Extract line points
        x1 = lines(k).point1(1);
        y1 = lines(k).point1(2);
        x2 = lines(k).point2(1);
        y2 = lines(k).point2(2);

        % Calculate the line length
        line_length = sqrt((x2 - x1)^2 + (y2 - y1)^2);

        % Calculate the angle of the line with respect to the horizontal
        angle = atan2d(y2 - y1, x2 - x1);

        % Normalize angle to 0°-180° range
        if angle < 0
            angle = angle + 180;
        end

        % Only consider diagonal lines (angles near 45° or 135°)
        if line_length >= min_length && ...
           ((abs(angle - 45) <= angle_tolerance) || (abs(angle - 135) <= angle_tolerance)) && ...
           line_length > min_endpoint_separation
            diagonal_lines = [diagonal_lines; [x1, y1, x2, y2, angle]];
        end
    end

    % Find intersections between diagonal lines
    x_intersections = [];
    y_intersections = [];

    for i = 1:size(diagonal_lines, 1)
        for j = i+1:size(diagonal_lines, 1)
            % Extract endpoints of two lines
            line1 = diagonal_lines(i, 1:4);
            line2 = diagonal_lines(j, 1:4);
            angle1 = diagonal_lines(i, 5);
            angle2 = diagonal_lines(j, 5);

            % Ensure the lines are roughly perpendicular (angle difference ~90°)
            angle_diff = abs(angle1 - angle2);
            if abs(angle_diff - 90) > angle_tolerance
                continue;
            end

            % Calculate intersection
            [x_int, y_int] = line_intersection([line1(1:2); line1(3:4)], [line2(1:2); line2(3:4)]);

            % Validate intersection (close to both lines)
            if ~isnan(x_int) && ~isnan(y_int)
                % Check if the intersection lies near both lines
                dist1 = point_to_line_distance([x_int, y_int], line1);
                dist2 = point_to_line_distance([x_int, y_int], line2);

                % Ensure intersection is geometrically within line segments
                if dist1 < 5 && dist2 < 5 && ...
                   is_within_line_segment([x_int, y_int], line1) && ...
                   is_within_line_segment([x_int, y_int], line2)
                    x_intersections = [x_intersections; x_int];
                    y_intersections = [y_intersections; y_int];
                end
            end
        end
    end

    % Combine intersections into Nx2 array
    detected_crosses = [x_intersections, y_intersections];

    % Filter overlapping crosses (within 10 pixels)
    if ~isempty(detected_crosses)
        % Perform clustering based on proximity
        clustered_crosses = [];
        while ~isempty(detected_crosses)
            % Take the first cross and find close ones
            ref_cross = detected_crosses(1, :);
            distances = sqrt(sum((detected_crosses - ref_cross).^2, 2));
            close_indices = find(distances <= 45);

            % Retain the first point within the overlap cluster
            retained_cross = detected_crosses(1, :);

            % Keep the centroid of the cluster as a representative point
            cluster = detected_crosses(close_indices, :);
            representative_cross = mean(cluster, 1);

            % Combine retained point and representative centroid
            clustered_crosses = [clustered_crosses; retained_cross];

            % Remove the clustered points
            detected_crosses(close_indices, :) = [];
        end

        % Return filtered crosses
        crosses = round(clustered_crosses);
    else
        crosses = [];
    end
end

% Function to calculate the intersection of two lines
function [x_int, y_int] = line_intersection(line1, line2)
    x1 = line1(1,1); y1 = line1(1,2);
    x2 = line1(2,1); y2 = line1(2,2);
    x3 = line2(1,1); y3 = line2(1,2);
    x4 = line2(2,1); y4 = line2(2,2);

    % Calculate denominators
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    % If lines are parallel, return NaN
    if denom == 0
        x_int = NaN;
        y_int = NaN;
        return;
    end

    % Calculate intersection
    x_int = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    y_int = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
end

% Function to calculate the distance from a point to a line segment
function dist = point_to_line_distance(point, line)
    x0 = point(1);
    y0 = point(2);
    x1 = line(1);
    y1 = line(2);
    x2 = line(3);
    y2 = line(4);

    num = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1);
    denom = sqrt((y2 - y1)^2 + (x2 - x1)^2);
    dist = num / denom;
end

% Function to check if a point is within a line segment
function is_within = is_within_line_segment(point, line)
    x0 = point(1);
    y0 = point(2);
    x1 = line(1);
    y1 = line(2);
    x2 = line(3);
    y2 = line(4);

    is_within = (min(x1, x2) <= x0 && x0 <= max(x1, x2)) && ...
                (min(y1, y2) <= y0 && y0 <= max(y1, y2));
end
