%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Robot Movement & Camera Testing - 5
%       Move, take picture of paper, and process image
%       Group 2, E24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;

% Load the appropriate library for the operating system
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
end

% Control table address
ADDR_MX_TORQUE_ENABLE        = 24;
ADDR_MX_CW_COMPLIANCE_MARGIN = 26;
ADDR_MX_CCW_COMPLIANCE_MARGIN = 27;
ADDR_MX_CW_COMPLIANCE_SLOPE  = 28;
ADDR_MX_CCW_COMPLIANCE_SLOPE = 29;
ADDR_MX_GOAL_POSITION        = 30;
ADDR_MX_MOVING_SPEED         = 32;
ADDR_MX_PRESENT_POSITION     = 36;
ADDR_MX_PUNCH                = 48;


% Protocol version
PROTOCOL_VERSION = 1.0;

% Default settings
DXL_IDS         = [1, 2, 3, 4];   % IDs of Dynamixels to control
BAUDRATE        = 1000000;
DEVICENAME      = 'COM8';         % Adjust as needed
TORQUE_ENABLE   = 1;
TORQUE_DISABLE  = 0;
START_POSITION  = 512;            % 150 degrees (starting position for all axes)
DXL_MOVING_STATUS_THRESHOLD = 10; % Position threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

% Define communication success and fail values
COMM_SUCCESS = 0;
COMM_TX_FAIL = -1001;

% Initialize PortHandler and PacketHandler
port_num = portHandler(DEVICENAME);
packetHandler();

% Open port and set baudrate
if ~openPort(port_num)
    unloadlibrary(lib_name);
    error('Failed to open port');
end

if ~setBaudRate(port_num, BAUDRATE)
    unloadlibrary(lib_name);
    error('Failed to set baudrate');
end

% Enable torque and set compliance/moving speed for each motor in DXL_IDS
for DXL_ID = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CW_COMPLIANCE_MARGIN, 0);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CCW_COMPLIANCE_MARGIN, 0);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CW_COMPLIANCE_SLOPE, 32);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CCW_COMPLIANCE_SLOPE, 32);
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_MOVING_SPEED, 100);
end

% Move all motors to the starting position of 150 degrees (511/512 in decimal)
START_POSITION = 512;
for DXL_ID = DXL_IDS
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, START_POSITION);
    pause(0.1); % Brief delay for each motor to start moving
end

% Wait for all motors to reach the start position
for DXL_ID = DXL_IDS
    while true
        dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_POSITION);
        if abs(START_POSITION - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD
            break;
        end
        pause(0.01); % Short delay for position checking
    end
end
fprintf('All motors have reached the start position.\n');

% Initialize Camera
camera_list = webcamlist();
disp('Available cameras:');
disp(camera_list);

% Select the first camera by name
cam = webcam('Logi C270 HD WebCam'); % Adjust name based on available cameras
disp('Camera initialized.');

% Move Axis 4 (DXL_ID = 4) to 60 degrees (position 205)
TARGET_POSITION_AXIS_4 = 205;
DXL_ID = 4;
write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, TARGET_POSITION_AXIS_4);

% Wait until Axis 4 reaches the target position
while true
    dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_POSITION);
    if abs(TARGET_POSITION_AXIS_4 - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD
        break;
    end
    pause(5); % Short delay for position checking
end
fprintf('Axis 4 has reached the target position of 60 degrees.\n');

% Set frame middle point based on assumed camera resolution
x_res = 640;
y_res = 480;
frame_middle = [x_res / 2, y_res / 2]; 

% Tic-Tac-Toe Board Tracking
fprintf('Detecting and tracking tic-tac-toe board...\n');

while true
    % Check for escape key input
    if ~isempty(get(gcf,'CurrentCharacter')) && strcmp(get(gcf,'CurrentCharacter'), ESC_CHARACTER)
        disp('Escape key pressed. Exiting loop.');
        break;
    end
    
    img = snapshot(cam);
    gray_img = rgb2gray(img);
    edges = edge(gray_img, 'Canny');

    % Perform Hough transform to detect lines
    [H, T, R] = hough(edges);
    P = houghpeaks(H, 8, 'threshold', ceil(0.3 * max(H(:)))); % Detect peaks for prominent lines
    lines = houghlines(edges, T, R, P, 'FillGap', 20, 'MinLength', 50);

    % Filter lines based on angles to detect the grid
    vertical_lines = lines(abs([lines.theta]) < 10 | abs([lines.theta] - 180) < 10);
    horizontal_lines = lines(abs([lines.theta] - 90) < 10);

    if length(vertical_lines) >= 2 && length(horizontal_lines) >= 2
        % Sort lines by their coordinates to get boundaries of middle square
        vertical_lines = sortrows(struct2table(vertical_lines), 'point1');
        horizontal_lines = sortrows(struct2table(horizontal_lines), 'point1');
        
        % Get middle square's estimated center coordinates
        middle_x = (vertical_lines.point1(1,1) + vertical_lines.point1(2,1)) / 2;
        middle_y = (horizontal_lines.point1(1,2) + horizontal_lines.point1(2,2)) / 2;
        center_square = [middle_x, middle_y];
        
        % Display detected grid lines and center of middle square
        figure(1), imshow(img), hold on;
        for k = 1:length(lines)
            xy = [lines(k).point1; lines(k).point2];
            plot(xy(:, 1), xy(:, 2), 'LineWidth', 2, 'Color', 'green');
        end
        plot(center_square(1), center_square(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        title('Tracking Tic-Tac-Toe Board');
        hold off;
        drawnow;

        % Calculate offsets between frame center and middle square center
        x_offset = frame_middle(1) - center_square(1);
        y_offset = frame_middle(2) - center_square(2);
        
        % Adjust robot's position based on these offsets
        delta_j1 = x_offset / 40; % Adjust sensitivity as needed
        delta_j4 = y_offset / 40; % Adjust sensitivity as needed

        % Retrieve current joint positions for precise adjustment
        current_j1_pos = read2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_PRESENT_POSITION);
        current_j4_pos = read2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_PRESENT_POSITION);
        
        % Calculate new goal positions
        new_j1_pos = current_j1_pos + round(delta_j1);
        new_j4_pos = current_j4_pos + round(delta_j4);
        
        % Send goal positions to Dynamixels
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 1, ADDR_MX_GOAL_POSITION, new_j1_pos);
        write2ByteTxRx(port_num, PROTOCOL_VERSION, 4, ADDR_MX_GOAL_POSITION, new_j4_pos);
        pause(0.1); % Allow time for movement
    else
        disp('Tic-tac-toe board not detected. Adjust the initial position and try again.');
    end
end

% Disable Dynamixel Torque for all motors
for DXL_ID = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
end


% Disable Dynamixel Torque for all motors
for DXL_ID = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
end

% Close port, release camera, and unload library
closePort(port_num);
clear cam; % Release camera
unloadlibrary(lib_name);

fprintf('Program terminated.\n');
