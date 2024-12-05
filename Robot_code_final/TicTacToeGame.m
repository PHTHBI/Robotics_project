classdef TicTacToeGame
    properties
        BoardDetector       % Instance of BoardDetector class
        Robot               % Robot object for interaction
        UserSymbol          % Symbol for the user ('X' or 'O')
        RobotSymbol         % Symbol for the robot ('X' or 'O')
        GameBoard           % 3x3 game board matrix
        cellCenters         % Detected cell centers on the board
        gridCorners         % Detected grid corners on the board
        MarkerDrawer        % For drawing X and O
        InverseKinematics   % Instance of InverseKinematics class
        PortNum            % Port number for robot communication
    end
    
    methods
        function obj = TicTacToeGame(robot, boardDetector, gridCorners, cellCenters, port_num)
            % Constructor: Initialize the game
            obj.Robot = robot;
            obj.BoardDetector = boardDetector;
            obj.GameBoard = zeros(3, 3); % Empty game board
            obj.gridCorners = gridCorners; % Initialize with pre-detected grid corners
            obj.cellCenters = cellCenters; % Initialize with pre-detected cell centers
            obj.MarkerDrawer = MarkerDrawer(obj.Robot); % Initialize MarkerDrawer
            obj.InverseKinematics = InverseKinematics(); % Initialize InverseKinematics
            obj.PortNum = port_num; % Store port number
        end

    
        function playGame(obj)
            % Main game loop
            disp('Game start! You go first.');
            obj.UserSymbol = input('Do you want to be X or O? ', 's');
            obj.RobotSymbol = 'X';
            if strcmp(obj.UserSymbol, 'X')
                obj.RobotSymbol = 'O';
            end
    
            isPlayerTurn = true;
    
            while true
                if isPlayerTurn
                    disp('Your turn. Make a move and press Enter.');
                    pause; % Wait for the user to make their move
                    obj.GameBoard = obj.detectPlayerMove();
                else
                    disp('Robot is making a move...');
                    obj.GameBoard = obj.robotMakeMove(); % Call robotMakeMove without passing board

                end
                %disp('Updated Game Board:');
                %disp(flipud(obj.GameBoard));
                % 
                obj.displayBoard();
    
                % Check for winner
                result = obj.checkWinner();
                if result ~= 0
                    if result == 1
                        disp('Congratulations! You win!');
                    elseif result == -1
                        disp('Robot wins!');
                    else
                        disp('It''s a tie!');
                    end
                    break;
                end
                isPlayerTurn = ~isPlayerTurn;
            end
        end
    
        function board = detectPlayerMove(obj,cellCenters)
            % Capture a new image of the board
            DXL_IDS = [1, 2, 3, 4]; % Dynamixel motor IDs

            resetPosition = [150, 150, 60, 150];
            obj.Robot.robotMove(obj.PortNum, resetPosition, DXL_IDS);

            pause(1);
            testPosition = [150, 140, 82, 65]; % Angles in degrees within your robot's limits
            obj.Robot.robotMove(obj.PortNum, testPosition, DXL_IDS);

            pause(2);
            obj.BoardDetector.captureBoardImage();
            
            % Read the updated image
            img = imread(obj.BoardDetector.ImagePath);
        
            % Display the updated image with grid and cell centers
            disp('Displaying the updated board with grid and cell centers...');
            imshow(img); hold on;
        
            % Retrieve pre-stored grid corners and cell centers
            cellCenters = obj.BoardDetector.cellCenters();
        
            % Plot cell centers
            plot(cellCenters(:, 1), cellCenters(:, 2), ...
                 'bo', 'MarkerFaceColor', 'b'); % Blue dots for cell centers
            hold off;
        
            % Process the player's move using the updated image
            board = obj.processPlayerMove(img);
        end



    
        function board = processPlayerMove(obj, img)
            % Detect player's move based on image processing
            disp('Processing player move...');
            
            % Convert the image to grayscale
            grayImage = rgb2gray(img);
        
            % --- Circle Detection (Os) ---
            % Apply Gaussian smoothing to reduce noise and smooth edges
            smoothedImage = imgaussfilt(grayImage, 1.5);
        
            % Enhance edges using adaptive histogram equalization
            enhancedImage = adapthisteq(smoothedImage);
        
            % Use edge detection for improved clarity
            edges = edge(enhancedImage, 'Canny', [0.1, 0.3]);
        
            % Detect circles using imfindcircles
            [centers, radii, metrics] = imfindcircles(edges, [15 200], ...
                'ObjectPolarity', 'bright', 'Sensitivity', 0.95, 'EdgeThreshold', 0.15);
        
            % Filter circles based on detection metric
            minMetric = 0.4; % Minimum detection metric threshold
            validIndices = metrics > minMetric;
            centers = centers(validIndices, :);
            radii = radii(validIndices);
        
            % --- Cross Detection (Xs) ---
            % Smooth the image to reduce noise
            smoothed_image = imgaussfilt(grayImage, 2);
        
            % Apply edge detection using the Canny method
            edges = edge(smoothed_image, 'Canny', [0.1, 0.3]);
        
            % Perform Hough Transform to detect lines
            [H, theta, rho] = hough(edges);
        
            % Find peaks in the Hough Transform
            peaks = houghpeaks(H, 20, 'Threshold', ceil(0.3 * max(H(:))));
        
            % Get lines based on Hough Transform peaks
            lines = houghlines(edges, theta, rho, peaks);
        
            % Initialize variables to store processed diagonal lines
            diagonal_lines = [];
            min_length = 5;  % Minimum line length constraint
            angle_tolerance = 10;  % Tolerance for angles around 45° and 135°
            min_endpoint_separation = 10;  % Minimum distance between endpoints for diagonal lines
        
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
                   sqrt((x2 - x1)^2 + (y2 - y1)^2) > min_endpoint_separation
                   diagonal_lines = [diagonal_lines; [x1, y1, x2, y2, angle]];
                end
            end
        
            % Find intersections between diagonal lines for the crosses
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
        
            % Update the game board
            figure, imshow(img); 
            hold on;
        
            % Draw the detected circles
            viscircles(centers, radii, 'EdgeColor', 'g', 'LineWidth', 1.5);
        
            % Mark the centers of the circles with red crosses
            plot(centers(:,1), centers(:,2), 'rx', 'LineWidth', 2, 'MarkerSize', 8);
        
            % Draw detected crosses (Xs)
            for i = 1:length(x_intersections)
                plot(x_intersections(i), y_intersections(i), 'bx', 'LineWidth', 2, 'MarkerSize', 8); % Blue X
            end
        
            % Save the updated image overwriting the old file
            saveas(gcf, obj.BoardDetector.ImagePath);
            disp(['Updated image saved to ', obj.BoardDetector.ImagePath]);
        
            for row = 1:3
                for col = 1:3
                    cellCenter = obj.cellCenters((row - 1) * 3 + col, :);
        
                    % Check for "O"
                    for i = 1:size(centers, 1)
                        if pdist2(centers(i, :), cellCenter) < 45
                            obj.GameBoard(row, col) = 1; % O detected
                        end
                    end
        
                    % Check for "X"
                    for i = 1:length(x_intersections)
                        if pdist2([x_intersections(i), y_intersections(i)], cellCenter) < 30
                            obj.GameBoard(row, col) = -1; % X detected
                        end
                    end
                end
            end
            board = obj.GameBoard;
        end

    
        function board = robotMakeMove(obj)
            % Robot makes its move
            %port_num = obj.Robot.port_num; % Retrieve port_num from the Robot instance;
            [row, col] = obj.getBestMove();
            obj.GameBoard(row, col) = -1; % Robot's move is -1
            disp(['Robot moves to: (', num2str(row), ', ', num2str(col), ')']);
            
            % Define the positions for the 9 cells
            markPosition = {
            [163, 130, 30, 120], [160, 119, 46, 120], [158, 113, 58, 115]; % Row 3
            [145, 130, 30, 120], [145, 122, 46, 115], [145, 113, 58, 110]; % Row 2
            [130, 130, 30, 120], [132, 121, 46, 115], [135, 115, 58, 112]; % Row 1
            };

            %Retrieve the joint angle
            jointAngles = markPosition{4 - row,col};
            disp(jointAngles);
            disp(['Moving to joint angles:',num2str(jointAngles)]);
            
            DXL_IDS = [1, 2, 3, 4]; % Dynamixel motor IDs

            loweredPosition = [150, 140, 60, 110];
            obj.Robot.robotMove(obj.PortNum, loweredPosition, DXL_IDS);
            
            pause(1);
            % Move the robot to the computed angles
            obj.Robot.robotMove(obj.PortNum, jointAngles, DXL_IDS);
            
           
            pause(3);
            
            capPosition = [140, 140, 82, 65];
            obj.Robot.robotMove(obj.PortNum, capPosition, DXL_IDS);
            disp('Please draw a x for me on my chosen location and press enter');
            pause;
            
            % Return the updated game board
            board = obj.GameBoard;
        end

    
        function [row, col] = getBestMove(obj)
            % Determines the robot's best move
            % Outputs:
            %   row, col - The row and column indices for the robot's next move
        
            % Get the current game board
            board = obj.GameBoard;
        
            % Check for a winning or blocking move
            row = 0; col = 0;
            for player = [-1, 1] % First check for robot (-1), then for player (1)
                for i = 1:3
                    % Check rows
                    if sum(board(i, :) == player) == 2 && any(board(i, :) == 0)
                        col = find(board(i, :) == 0, 1);
                        row = i;
                        return; % Found a winning or blocking move
                    end
        
                    % Check columns
                    if sum(board(:, i) == player) == 2 && any(board(:, i) == 0)
                        row = find(board(:, i) == 0, 1);
                        col = i;
                        return; % Found a winning or blocking move
                    end
                end
        
                % Check main diagonal
                if sum(diag(board) == player) == 2 && any(diag(board) == 0)
                    idx = find(diag(board) == 0, 1);
                    row = idx;
                    col = idx;
                    return; % Found a winning or blocking move
                end
        
                % Check anti-diagonal
                if sum(diag(flipud(board)) == player) == 2 && any(diag(flipud(board)) == 0)
                    idx = find(diag(flipud(board)) == 0, 1);
                    row = idx;
                    col = 4 - idx; % Flip index for anti-diagonal
                    return; % Found a winning or blocking move
                end
            end
        
            % If no winning or blocking move, choose a random empty square
            emptySquares = find(board == 0); % Find all empty cells
            if isempty(emptySquares)
                error('No empty squares available on the board.'); % Ensure there are moves left
            end
        
            idx = randi(length(emptySquares)); % Randomly select one of the empty squares
            [row, col] = ind2sub(size(board), emptySquares(idx)); % Convert linear index to row, col
        end


        function displayBoard(obj)
            % Display the current state of the board
            disp('Current Board:');
            disp(obj.GameBoard);
        end
    
        function result = checkWinner(obj)
            % Checks the current board state for a winner or a tie
            %   result - Indicator of the game outcome:
            %            1  -> Player wins
            %           -1  -> Robot wins
            %            0  -> No winner yet
            %            2  -> Tie (all cells filled with no winner)
            
            % Use the object's GameBoard as the board to check
            board = obj.GameBoard;
            
            % Check rows and columns for a winner
            for i = 1:3
                % Check rows
                if all(board(i, :) == 1)
                    result = 1; % Player wins
                    return;
                elseif all(board(i, :) == -1)
                    result = -1; % Robot wins
                    return;
                end
        
                % Check columns
                if all(board(:, i) == 1)
                    result = 1; % Player wins
                    return;
                elseif all(board(:, i) == -1)
                    result = -1; % Robot wins
                    return;
                end
            end
        
            % Check main diagonal
            if all(diag(board) == 1)
                result = 1; % Player wins
                return;
            elseif all(diag(board) == -1)
                result = -1; % Robot wins
                return;
            end
        
            % Check anti-diagonal
            if all(diag(flipud(board)) == 1)
                result = 1; % Player wins
                return;
            elseif all(diag(flipud(board)) == -1)
                result = -1; % Robot wins
                return;
            end
        
            % Check for a tie (all cells filled with no winner)
            if all(board(:) ~= 0)
                result = 2; % Tie
                return;
            end
        
            % No winner yet
            result = 0;
        end

    end
end
