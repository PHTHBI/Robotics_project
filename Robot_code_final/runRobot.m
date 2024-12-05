% Load the appropriate library for the operating system
clc;
clear;

% Create an instance of the RobotController
robot = ConnectRobot;

if libisloaded('dxl_x64_c')
    unloadlibrary('dxl_x64_c');
    disp('unloaded running library')
end

lib_name = '';
if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
    disp('Library loaded successfully.');
end

% Create an instance of the CameraCalibration class
%cameraCalib = CalibrateCamera();


% Initialize Camera
camera_list = webcamlist();
disp('Available cameras:');
disp(camera_list);

% Select the first camera by name
cam = webcam('Logi C270 HD WebCam'); % Adjust name based on available cameras
disp('Camera initialized.');



% Create an instance of the TicTacToeDetector class
ticTacToeImagePath = "C:\Users\nanna\OneDrive\Skrivebord\MSc E24\Robotics\DynamixelSDK-3.7.31\DynamixelSDK-3.7.31\matlab\TicTacToe.jpg";
boardDetector = BoardDetector(ticTacToeImagePath);


try
    % Connect to the robot
    disp('Connecting to robot...');
    port_num = robot.robotConnect('COM6', 'joint'); % Replace 'COM6' with your actual port
    disp('Robot connected successfully.');

    % Move the robot to start position
    disp('Moving robot to start position...');
    testPosition = [150, 150, 60, 150]; % Angles in degrees within your robot's limits
    DXL_IDS = [1, 2, 3, 4];
    robot.robotMove(port_num, testPosition, DXL_IDS);
    disp('Robot moved to start position.');

    %% Move the robot to calibration position
    disp('Moving robot to test position...');
    testPosition = [150, 140, 82, 65]; % Angles in degrees within your robot's limits
    robot.robotMove(port_num, testPosition, DXL_IDS);
    disp('Robot moved to test position.');

   

    %% Capture an image of the tic-tac-toe board
    pause(2);
    disp('Capturing tic-tac-toe board image...');
    boardDetector.captureBoardImage(); % Capture the tic-tac-toe board image
    disp(['Image saved to ', ticTacToeImagePath]);
    
    % Detect grid intersections and store them in the object
    disp('Detecting grid intersections...');
    boardDetector.gridCorners = boardDetector.detectFourIntersections();
    disp('Grid Corners:');
    disp(boardDetector.gridCorners);
    
    % Detect grid cell centers and store them in the object
    disp('Detecting grid cell centers...');
    boardDetector.cellCenters = boardDetector.findGridCellCenters(boardDetector.gridCorners);
    disp('Cell Centers:');
    disp(boardDetector.cellCenters);

    % Display the results on the captured image
    img = imread(ticTacToeImagePath);
    imshow(img);
    hold on;
    plot(boardDetector.gridCorners(:, 1), boardDetector.gridCorners(:, 2), 'ro', 'MarkerFaceColor', 'r'); % Red dots for grid corners
    plot(boardDetector.cellCenters(:, 1), boardDetector.cellCenters(:, 2), 'bo', 'MarkerFaceColor', 'b'); % Blue dots for cell centers
    hold off;



    %% Setup the Tic Tac Toe board and cell centers
    % Create an instance of the TicTacToeGame class with pre-detected grid and cell centers
    ticTacToeGame = TicTacToeGame(robot, boardDetector, boardDetector.gridCorners, boardDetector.cellCenters, port_num);

    % Play the game
    ticTacToeGame.playGame();

    %% Terminate the connection
    disp('Moving robot to start position...');
    testPosition = [150, 150, 60, 150]; % Angles in degrees within your robot's limits
    DXL_IDS = [1, 2, 3, 4];
    robot.robotMove(port_num, testPosition, DXL_IDS);
    disp('Robot moved to start position.');
    
    disp('Terminating robot connection...');
    robot.robotTerminate(port_num, [1, 2, 3, 4], 1);
    disp('Robot connection terminated.');

    unloadlibrary('dxl_x64_c');
    clear cam;

catch ME
    % Handle errors gracefully
    disp('An error occurred:');
    disp(ME.message);

    % Attempt to terminate the robot connection in case of failure
    if exist('portHandler', 'var')
        robot.robotTerminate(port_num, [1, 2, 3, 4], 1);
    end
end