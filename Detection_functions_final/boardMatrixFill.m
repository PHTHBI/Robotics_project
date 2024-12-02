function board = boardMatrixFill(square_centers, cross_centers, circle_centers)
    % FILLTICTACTOEBOARD fills a 3x3 matrix based on detected square centers,
    % cross centers (X), and circle centers (O).
    % Input:
    %   - square_centers: 9x2 array of square centers (ordered left to right, top to bottom)
    %   - cross_centers: Mx2 array of cross (X) centers
    %   - circle_centers: Kx2 array of circle (O) centers
    % Output:
    %   - board: 3x3 matrix representing the Tic Tac Toe board
    %       0 = empty
    %       1 = X
    %      -1 = O

    % Initialize the board as empty
    board = zeros(3, 3);
    acceptable_error = 35; % Acceptable error in position matching

    % Loop through each square center (9 total)
    for i = 1:9
        % Get the (x, y) position of the current square
        square_center = square_centers(i, :);
        
        % Calculate the row and column indices in the 3x3 board
        row = floor((i - 1) / 3) + 1; % Determine the row (1, 2, or 3)
        col = mod(i - 1, 3) + 1;       % Determine the column (1, 2, or 3)

        % Check for a matching cross (X) in proximity
        if ~isempty(cross_centers)
            distances_to_crosses = sqrt(sum((cross_centers - square_center).^2, 2));
            if any(distances_to_crosses <= acceptable_error)
                board(row, col) = 1; % Assign X to the square
                continue;
            end
        end

        % Check for a matching circle (O) in proximity
        if ~isempty(circle_centers)
            distances_to_circles = sqrt(sum((circle_centers - square_center).^2, 2));
            if any(distances_to_circles <= acceptable_error)
                board(row, col) = -1; % Assign O to the square
                continue;
            end
        end

        % If no match, leave the square as empty (0)
        board(row, col) = 0;
    end

    % Display the current board
    dispBoard(board);
end

% Function to display the Tic Tac Toe board
function dispBoard(board)
    % Convert board matrix to characters for display
    dispBoard = char(board + ' '); % Convert numbers to characters
    dispBoard(board == 1) = 'X';
    dispBoard(board == -1) = 'O';
    dispBoard(board == 0) = '.';

    % Display the board
    fprintf('\nCurrent Board:\n');
    fprintf(' %c | %c | %c\n', dispBoard(1,1), dispBoard(1,2), dispBoard(1,3));
    fprintf('---+---+---\n');
    fprintf(' %c | %c | %c\n', dispBoard(2,1), dispBoard(2,2), dispBoard(2,3));
    fprintf('---+---+---\n');
    fprintf(' %c | %c | %c\n\n', dispBoard(3,1), dispBoard(3,2), dispBoard(3,3));
end
