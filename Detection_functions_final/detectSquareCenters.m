function square_centers = detectSquareCenters(image)
    % Detect square centers in a Tic-Tac-Toe board from the input image
    
    % Step 1: Convert to grayscale if not already
    if size(image, 3) == 3
        image = rgb2gray(image);
    end

    % Step 2: Enhance contrast for better line detection
    enhanced_image = imadjust(image); % Adjust image intensity values

    % Step 3: Apply edge detection
    edges = edge(enhanced_image, 'Canny');

    % Step 4: Detect lines using the Hough Transform
    [H, theta, rho] = hough(edges);

    % Step 5: Identify peaks in the Hough transform
    peaks = houghpeaks(H, 20, 'threshold', ceil(0.3 * max(H(:))));

    % Step 6: Extract lines based on the Hough transform
    lines = houghlines(edges, theta, rho, peaks, 'FillGap', 10, 'MinLength', 20);

    % Step 7: Filter vertical and horizontal lines
    [vertical_lines, horizontal_lines] = filter_lines(lines);

    % Step 8: Compute grid intersection points
    grid_points = compute_intersections(vertical_lines, horizontal_lines);

    % Step 9: Calculate and return the centers of all squares if enough points are detected
    if size(grid_points, 1) >= 4
        square_centers = calculate_square_centers(grid_points);
    else
        square_centers = []; % Return empty if grid points not detected correctly
    end
end

% --- Helper Functions ---

% Function: Filter vertical and horizontal lines
function [vertical_lines, horizontal_lines] = filter_lines(lines)
    vertical_lines = [];
    horizontal_lines = [];
    for k = 1:length(lines)
        % Calculate the angle of each line
        angle = atan2d(lines(k).point2(2) - lines(k).point1(2), lines(k).point2(1) - lines(k).point1(1));
        if angle < 0
            angle = angle + 180; % Normalize angle to [0, 180]
        end

        % Classify the lines
        if abs(angle - 90) <= 5  % Vertical lines
            vertical_lines = [vertical_lines; lines(k)];
        elseif abs(angle) <= 5 || abs(angle - 180) <= 5  % Horizontal lines
            horizontal_lines = [horizontal_lines; lines(k)];
        end
    end
end

% Function: Compute intersections between vertical and horizontal lines
function grid_points = compute_intersections(vertical_lines, horizontal_lines)
    x_coords = [];
    y_coords = [];
    for i = 1:length(vertical_lines)
        for j = 1:length(horizontal_lines)
            [x, y] = line_intersection(vertical_lines(i), horizontal_lines(j));
            if ~isempty(x) && ~isempty(y)
                x_coords = [x_coords; x];
                y_coords = [y_coords; y];
            end
        end
    end
    % Remove duplicate points and sort
    grid_points = unique([x_coords, y_coords], 'rows');
    grid_points = sortrows(grid_points, [2, 1]); % Sort by y, then x
end

% Function: Calculate centers of all squares
function square_centers = calculate_square_centers(grid_points)
    % Compute grid dimensions
    min_x = min(grid_points(:, 1));
    max_x = max(grid_points(:, 1));
    min_y = min(grid_points(:, 2));
    max_y = max(grid_points(:, 2));
    
    % Calculate square dimensions
    square_width = (max_x - min_x);
    square_height = (max_y - min_y);

    % Calculate centers for all 9 squares
    center_x = (min_x + max_x) / 2;
    center_y = (min_y + max_y) / 2;
    square_centers = [];
    for row_offset = -1:1 % Start from 1 (top row), go to -1 (bottom row)
        for col_offset = -1:1 % Start from -1 (left column), go to 1 (right column)
        square_centers = [square_centers; center_x + col_offset * square_width, ...
                                          center_y + row_offset * square_height];
        end
    end
end


% Function: Compute line intersection
function [x, y] = line_intersection(line1, line2)
    % Extract endpoints
    x1 = line1.point1(1); y1 = line1.point1(2);
    x2 = line1.point2(1); y2 = line1.point2(2);
    x3 = line2.point1(1); y3 = line2.point1(2);
    x4 = line2.point2(1); y4 = line2.point2(2);
    
    % Solve for intersection
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if denom == 0
        x = [];
        y = [];
    else
        x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
        y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;
    end
end
