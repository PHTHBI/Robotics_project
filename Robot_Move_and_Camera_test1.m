%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Robot Movement & Camera Testing - 1
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
DEVICENAME      = 'COM8'; % Adjust as needed
TORQUE_ENABLE   = 1;
TORQUE_DISABLE  = 0;
DXL_MIN_POS     = 70;             % Minimum position value
DXL_MAX_POS     = 963;            % Maximum position value
DXL_MOVING_STATUS_THRESHOLD = 10; % Position threshold


ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

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

% Set up for movement cycling between min and max positions for DXL_ID = 1
dxl_goal_position = [DXL_MIN_POS DXL_MAX_POS];
index = 1;
dxl_present_position = 0;

% Movement Loop
fprintf('Starting movement loop...\n');
while true
    if input('Press any key to continue! (or input e to quit!)\n', 's') == 'e'
        break;
    end

    % Write goal position for DXL_ID = 1
    DXL_ID = 1;
    write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position(index));
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('TxRx Result Error: %s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('Packet Error: %s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end

    % Wait until the motor reaches the target position
    while true
        dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_POSITION);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('TxRx Result Error: %s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('Packet Error: %s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        fprintf('[ID:%03d] GoalPos:%03d  PresPos:%03d\n', DXL_ID, dxl_goal_position(index), dxl_present_position);

        if ~(abs(dxl_goal_position(index) - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)
            break;
        end
    end

    % Toggle goal position index
    index = 3 - index; % Alternates between 1 and 2
end

% Disable Dynamixel Torque for all motors
for DXL_ID = DXL_IDS
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
end

% Close port and unload library
closePort(port_num);
unloadlibrary(lib_name);

fprintf('Program terminated.\n');
