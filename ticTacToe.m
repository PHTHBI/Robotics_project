function ticTacToe
    % Initialize the board
    board = zeros(3); % 3x3 grid, 0 = empty, 1 = player, -1 = computer
    dispBoard(board);
    
    % Decide who goes first
    firstPlayer = input('Do you want to go first? (1 = Yes, 0 = No): ');
    if firstPlayer
        playerTurn = true;
    else
        playerTurn = false;
    end
    
    % Main game loop
    while true
        if playerTurn
            % Player's turn
            [row, col] = getPlayerMove(board);
            board(row, col) = 1; % Player is represented by 1
        else
            % Computer's turn
            [row, col] = getComputerMove(board);
            board(row, col) = -1; % Computer is represented by -1
            fprintf('Computer played at (%d, %d)\n', row, col);
        end
        
        % Display the updated board
        dispBoard(board);
        
        % Check for win, lose, or draw
        result = checkWinner(board);
        if result == 1
            disp('You win!');
            break;
        elseif result == -1
            disp('Computer wins!');
            break;
        elseif result == 0 && all(board(:) ~= 0)
            disp('It''s a tie!');
            break;
        end
        
        % Switch turns
        playerTurn = ~playerTurn;
    end
end

% Display the current board
function dispBoard(board)
    dispBoard = char(board + ' '); % Convert numbers to characters
    dispBoard(board == 1) = 'X';
    dispBoard(board == -1) = 'O';
    dispBoard(board == 0) = '.';
    fprintf('\nCurrent Board:\n');
    fprintf(' %c | %c | %c\n', dispBoard(1,1), dispBoard(1,2), dispBoard(1,3));
    fprintf('---+---+---\n');
    fprintf(' %c | %c | %c\n', dispBoard(2,1), dispBoard(2,2), dispBoard(2,3));
    fprintf('---+---+---\n');
    fprintf(' %c | %c | %c\n\n', dispBoard(3,1), dispBoard(3,2), dispBoard(3,3));
end

% Get player move
function [row, col] = getPlayerMove(board)
    while true
        row = input('Enter row (1-3): ');
        col = input('Enter column (1-3): ');
        if row >= 1 && row <= 3 && col >= 1 && col <= 3 && board(row, col) == 0
            break;
        else
            disp('Invalid move. Try again.');
        end
    end
end

% Computer move decision-making
function [row, col] = getComputerMove(board)
    emptySquares = find(board == 0);
    
    % Check for a winning move first
    for i = 1:length(emptySquares)
        [r, c] = ind2sub([3, 3], emptySquares(i));
        
        % Check if computer can win
        board(r, c) = -1;
        if checkWinner(board) == -1
            row = r;
            col = c;
            return;
        end
        board(r, c) = 0;
    end
    
    % If no winning move, check if the computer needs to block the player's winning move
    for i = 1:length(emptySquares)
        [r, c] = ind2sub([3, 3], emptySquares(i));
        
        % Check if it needs to block player's winning move
        board(r, c) = 1;
        if checkWinner(board) == 1
            row = r;
            col = c;
            return;
        end
        board(r, c) = 0;
    end
    
    % If no winning or blocking move, choose a random empty square
    idx = randi(length(emptySquares));
    [row, col] = ind2sub([3, 3], emptySquares(idx));
end

% Check for a winner or draw
function result = checkWinner(board)
    % Check rows, columns, and diagonals for a win
    for i = 1:3
        if all(board(i, :) == 1) || all(board(:, i) == 1) % Player wins
            result = 1;
            return;
        elseif all(board(i, :) == -1) || all(board(:, i) == -1) % Computer wins
            result = -1;
            return;
        end
    end
    
    % Check diagonals
    if all(diag(board) == 1) || all(diag(flipud(board)) == 1)
        result = 1;
        return;
    elseif all(diag(board) == -1) || all(diag(flipud(board)) == -1)
        result = -1;
        return;
    end
    
    % No winner yet
    result = 0;
end
