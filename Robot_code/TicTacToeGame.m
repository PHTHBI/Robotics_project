classdef TicTacToeGame
    properties
        BoardDetector       % Instance of BoardDetector class
        BoardGrid           % Grid points on the Tic Tac Toe board
        Robot               % Robot object for interaction
        UserSymbol          % Symbol for the user ('X' or 'O')
        RobotSymbol         % Symbol for the robot ('X' or 'O')
        GameBoard           % 3x3 game board matrix
        CellCenters         % Detected cell centers on the board
        MarkerDrawer        % For drawing X and O
    end
    
    methods
        function obj = TicTacToeGame(robot, boardDetector)
            % Constructor: Initialize the game
            obj.Robot = robot;
            obj.BoardDetector = boardDetector;
            obj.GameBoard = zeros(3, 3); % Empty game board
    
            % Initialize the MarkerDrawer for drawing X and O
            obj.MarkerDrawer = MarkerDrawer(obj.Robot);
        end
    
        function obj = setupBoard(obj)
            % Capture board image and detect grid
            obj.BoardDetector.captureBoardImage();
            disp('Detecting grid intersections...');
            gridCorners = obj.BoardDetector.detectFourIntersections();
            disp('Grid corners detected.');
        
            % Detect cell centers
            disp('Detecting cell centers...');
            obj.CellCenters = obj.BoardDetector.findGridCellCenters(gridCorners);
            disp('Cell centers detected.');
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
                    obj.GameBoard = obj.robotMakeMove();
                end
    
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
    
            obj.Robot.moveToStartPosition();
            obj.Robot.disconnectHardware();
        end
    
        function board = detectPlayerMove(obj)
            % Capture image and detect the player's move
            img = imread(obj.BoardDetector.ImagePath);
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
            for row = 1:3
                for col = 1:3
                    cellCenter = obj.CellCenters((row - 1) * 3 + col, :);
        
                    % Check for "O"
                    for i = 1:size(centers, 1)
                        if pdist2(centers(i, :), cellCenter) < 20
                            obj.GameBoard(row, col) = 1; % O detected
                        end
                    end
        
                    % Check for "X"
                    for i = 1:length(x_intersections)
                        if pdist2([x_intersections(i), y_intersections(i)], cellCenter) < 20
                            obj.GameBoard(row, col) = -1; % X detected
                        end
                    end
                end
            end
            board = obj.GameBoard;
        end

    
        function board = robotMakeMove(obj)
            % Robot makes its move
            [row, col] = obj.getBestMove();
            obj.GameBoard(row, col) = -1; % Robot's move is -1
            disp(['Robot moves to: (', num2str(row), ', ', num2str(col), ')']);
            
            % Get the center of the selected cell
            cellCenter = obj.CellCenters((row - 1) * 3 + col, :);
            
            % Use the MarkerDrawer class to draw the robot's symbol
            if strcmp(obj.RobotSymbol, 'X')
                % Draw an X
                obj.MarkerDrawer.drawX(cellCenter); % Uses MarkerDrawer's drawX method
            else
                % Draw an O
                obj.MarkerDrawer.drawO(cellCenter); % Uses MarkerDrawer's drawO method
            end
        
            % Return the updated game board
            board = obj.GameBoard;
        end
    
        function [row, col] = getBestMove(obj, board)
            % Determines the robot's best move
            % Inputs:
            %   board - 3x3 matrix representing the current state of the game board
            %           0 -> empty cell
            %           1 -> player's symbol
            %          -1 -> robot's symbol
            % Outputs:
            %   row, col - The row and column indices for the robot's next move
        
            % Step 1: Check for a winning move
            [row, col] = obj.findWinningMove(board, -1); % -1 represents the robot
            if row > 0
                return;
            end
        
            % Step 2: Check for a blocking move
            [row, col] = obj.findWinningMove(board, 1); % 1 represents the player
            if row > 0
                return;
            end
        
            % Step 3: Prioritize the center if available
            if board(2, 2) == 0
                row = 2;
                col = 2;
                return;
            end
        
            % Step 4: Prioritize the corners
            corners = [1, 1; 1, 3; 3, 1; 3, 3];
            for i = 1:size(corners, 1)
                if board(corners(i, 1), corners(i, 2)) == 0
                    row = corners(i, 1);
                    col = corners(i, 2);
                    return;
                end
            end
        
            % Step 5: Choose any random empty square
            [emptyRows, emptyCols] = find(board == 0);
            idx = randi(length(emptyRows)); % Randomly select one of the empty squares
            row = emptyRows(idx);
            col = emptyCols(idx);
        end
    
        function [row, col] = findWinningMove(~, board, player)
        % Checks for a winning move for the given player
        % Inputs:
        %   board - 3x3 matrix representing the game board
        %   player - The player to check for (1 for user, -1 for robot)
        % Outputs:
        %   row, col - The row and column indices of the winning move

            for i = 1:3
                % Check rows
                if sum(board(i, :) == player) == 2 && any(board(i, :) == 0)
                    col = find(board(i, :) == 0, 1);
                    row = i;
                    return;
                end
    
                % Check columns
                if sum(board(:, i) == player) == 2 && any(board(:, i) == 0)
                    row = find(board(:, i) == 0, 1);
                    col = i;
                    return;
                end
            end
    
            % Check main diagonal
            if sum(diag(board) == player) == 2 && any(diag(board) == 0)
                idx = find(diag(board) == 0, 1);
                row = idx;
                col = idx;
                return;
            end
    
            % Check anti-diagonal
            if sum(diag(flipud(board)) == player) == 2 && any(diag(flipud(board)) == 0)
                idx = find(diag(flipud(board)) == 0, 1);
                row = idx;
                col = 4 - idx; % Flip index for anti-diagonal
                return;
            end
    
            % No winning move found
            row = 0;
            col = 0;
        end


        function displayBoard(obj)
            % Display the current state of the board
            disp('Current Board:');
            disp(obj.GameBoard);
        end
    
        function result = checkWinner(~, board)
            % Checks the current board state for a winner or a tie
            %   result - Indicator of the game outcome:
            %            1  -> Player wins
            %           -1  -> Robot wins
            %            0  -> No winner yet
            %            2  -> Tie (all cells filled with no winner)
        
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
