%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Robot Movement & Camera Testing - 3
%       Move and take picture of paper
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

% Capture image from the camera
img = snapshot(cam);
imshow(img);
title('Captured Image');
drawnow; % Update the display immediately

% Disable Dynamixel Torque for all motors
for DXL_ID = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
end

% Close port, release camera, and unload library
closePort(port_num);
clear cam; % Release camera
unloadlibrary(lib_name);

fprintf('Program terminated.\n');
