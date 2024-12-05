% Connects to robot and primes motors
classdef ConnectRobot
    properties (Constant)
        % Define constants
        ADDR_MX_TORQUE_ENABLE = 24;
        ADDR_MX_CW_COMPLIANCE_MARGIN = 26;
        ADDR_MX_CCW_COMPLIANCE_MARGIN = 27;
        ADDR_MX_CW_COMPLIANCE_SLOPE = 28;
        ADDR_MX_CCW_COMPLIANCE_SLOPE = 29;
        ADDR_MX_GOAL_POSITION = 30;
        ADDR_MX_MOVING_SPEED = 32;
        ADDR_MX_PRESENT_POSITION = 36;
        ADDR_MX_PUNCH = 48;
        ADDR_MOVING = 46;
        PROTOCOL_VERSION = 1.0;
        
        ADDR_CW_ANGLE_LIMIT = 6;
        ADDR_CCW_ANGLE_LIMIT = 8;
        
        TORQUE_ENABLE = 1;
        TORQUE_DISABLE = 0;
        
        % Standard values for setting the motors
        stdMargin = 10;
        stdSlope = 32;
        stdSpeed = 80;
        stdIDS = [1, 2, 3, 4];
        stdPunch = 32;
        
        % Set angle limits, pre-defined values
        DPU = 300 / 1023; % Degrees per unit of position
        %J1min = 60 / (300 / 1023);
        %J1max = 220 / (300 / 1023);
        %J2min = 20 / (300 / 1023);
        %J2max = 280 / (300 / 1023);
        %J3min = 145 / (300 / 1023);
        %J3max = 200 / (300 / 1023);
        %Bmin = 1 / (300 / 1023);
        %Bmax = 300 / (300 / 1023);
    end

    methods
        
        function port_num = robotConnect(obj, port, mode, speed, slope, margin, punch, DXL_IDS)
            % Default arguments
            if nargin < 8
                DXL_IDS = obj.stdIDS;
            end
            if nargin < 7
                punch = obj.stdPunch;
            end
            if nargin < 6
                margin = obj.stdMargin;
            end
            if nargin < 5
                slope = obj.stdSlope;
            end
            if nargin < 4
                speed = obj.stdSpeed;
            end
            if nargin < 3
                mode = 'joint';
            end
        
            DEVICENAME = port;
            BAUDRATE = 1000000;
        
            port_num = portHandler(DEVICENAME);
            packetHandler();
        
            if openPort(port_num)
                disp('Succeeded to open the port');
            else
                disp('Failed to open the port');
                return;
            end
        
            % Initialize Angle limits
            %mins = [obj.Bmin, obj.J1min, obj.J2min, obj.J3min];
            %maxes = [obj.Bmax, obj.J1max, obj.J2max, obj.J3max];
        
            % Set angle limits based on mode
            %for i = 1:length(DXL_IDS)
            %   if strcmp(mode, 'joint')
            %        write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_CW_ANGLE_LIMIT, round(mins(i)));
            %        write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_CCW_ANGLE_LIMIT, round(maxes(i)));
            %    else
            %        write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_CW_ANGLE_LIMIT, 0);
            %        write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_CCW_ANGLE_LIMIT, 0);
            %    end
            %end
        
            % Set port baudrate
            if setBaudRate(port_num,BAUDRATE)
                disp('Succeeded to change the baudrate');
            else
                disp('Failed to change the baudrate');
                return;
            end
        
            % Setting parameters
            disp('Setting robot parameters...');
            for i = 1:length(DXL_IDS)
                write1ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_TORQUE_ENABLE, obj.TORQUE_ENABLE);
                write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_CW_COMPLIANCE_MARGIN, margin);
                write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_CCW_COMPLIANCE_MARGIN, margin);
                write1ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_CW_COMPLIANCE_SLOPE, slope);
                write1ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_CCW_COMPLIANCE_SLOPE, slope);
                write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_MOVING_SPEED, speed);
                write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_PUNCH, punch);
            end
        end
        
        function robotMove(obj, port_num, pos, DXL_IDS)
            if nargin < 4
                DXL_IDS = obj.stdIDS;
            end
        
            motor_pos = pos / obj.DPU;
            motor_pos = round(motor_pos);
        
            for i = length(DXL_IDS):-1:1
                write2ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_GOAL_POSITION, motor_pos(i));
            end
        
            movingArr = ones(1, length(DXL_IDS));
        
            
            while sum(movingArr) > 0 % Check if any motor is still moving
                for i = length(DXL_IDS):-1:1 % Check status in reverse order
                    % Check the moving status of the motor
                    movingArr(i) = read1ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MOVING);
                end
            end
           
        end
        
        function robotTerminate(obj, port_num, DXL_IDS, disTorque)
            if nargin < 4
                disTorque = 0;
            end
            if nargin < 3
                DXL_IDS = obj.stdIDS;
            end
        
            if disTorque
                disp('Disabling torque');
                for i = 1:length(DXL_IDS)
                    write1ByteTxRx(port_num, obj.PROTOCOL_VERSION, DXL_IDS(i), obj.ADDR_MX_TORQUE_ENABLE, obj.TORQUE_DISABLE);
                end
            end
        
            % Close port
            closePort(port_num);
        end
    end
end

