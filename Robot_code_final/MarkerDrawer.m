classdef MarkerDrawer
    properties
        InverseKinematics % Instance of the InverseKinematics class
        Robot             % Instance of the RobotController class
        MarkerOffset      % Offset of the marker from the robot’s end effector
        penLiftHeight     % Height to lift pen between positions
        penDownHeight     % Height when pen is touching the board
    end

    methods
        function obj = MarkerDrawer(robot)
            % Constructor: Initialize with robot instance and marker offset
            obj.Robot = robot;
            obj.penLiftHeight = 120; % Lift height in mm (adjust based on robot calibration)
            obj.penDownHeight = 98; % Down height in mm (pen touching the paper)
            %obj.InverseKinematics = InverseKinematics();
            %obj.MarkerOffset = [30, 30, 0]; % [mm] Offset for the marker
        end

        function drawX(obj, jointAngles, port_num, DXL_IDS)
            % Draws an 'X' at the specified cell using its joint angles
            disp('Drawing X...');
            disp(['Starting position: ', num2str(jointAngles)]);
        
            % Define offsets for the diagonal lines in joint space (approx.)
            offset = 15; % Offset for drawing in mm
            diagonal1Start = [jointAngles(1), jointAngles(2) + offset, jointAngles(3), jointAngles(4)];
            diagonal1End = [jointAngles(1), jointAngles(2) - offset, jointAngles(3), jointAngles(4)];
        
            diagonal2Start = [jointAngles(1), jointAngles(2), jointAngles(3) + offset, jointAngles(4)];
            diagonal2End = [jointAngles(1), jointAngles(2), jointAngles(3) - offset, jointAngles(4)];
        
            % Draw the first diagonal
            obj.Robot.robotMove(port_num, diagonal1Start, DXL_IDS); % Move to start of diagonal 1
            obj.Robot.robotMove(port_num, jointAngles, DXL_IDS);   % Draw the line
            obj.Robot.robotMove(port_num, diagonal1End, DXL_IDS); % Move to end of diagonal 1
        
            % Lift the pen
            liftPosition = [jointAngles(1), jointAngles(2), obj.penLiftHeight, jointAngles(4)];
            obj.Robot.robotMove(port_num, liftPosition, DXL_IDS);
        
            % Draw the second diagonal
            obj.Robot.robotMove(port_num, diagonal2Start, DXL_IDS); % Move to start of diagonal 2
            obj.Robot.robotMove(port_num, jointAngles, DXL_IDS);    % Draw the line
            obj.Robot.robotMove(port_num, diagonal2End, DXL_IDS);  % Move to end of diagonal 2
        
            % Lift the pen again
            obj.Robot.robotMove(port_num, liftPosition, DXL_IDS);

            % go to capture board position
            capPosition = [150, 140, 82, 65];
            obj.Robot.robotMove(port_num, capPosition, DXL_IDS);
        
            disp('X drawn.');
        end


        function drawO(obj, center)
            % Draws an 'O' at the specified center position
            disp(['Drawing O at center: ', num2str(center)]);

            % Define parameters for the circle
            radius = 15; % Radius of the O in mm
            numPoints = 20; % Number of points to approximate the circle
            theta = linspace(0, 2 * pi, numPoints); % Angles for the circle

            % Generate points for the circle
            circlePoints = [center(1) + radius * cos(theta)', ...
                            center(2) + radius * sin(theta)', ...
                            repmat(obj.penDownHeight, numPoints, 1)];

            % Move to the starting position
            obj.moveToPosition([circlePoints(1, 1), circlePoints(1, 2), obj.penLiftHeight]);
            obj.moveToPosition(circlePoints(1, :)); % Lower to start drawing

            % Draw the circle
            for i = 2:numPoints
                obj.moveToPosition(circlePoints(i, :));
            end

            % Lift the pen at the end
            obj.moveToPosition([circlePoints(end, 1), circlePoints(end, 2), obj.penLiftHeight]);
        end

        function moveToPosition(obj, position)
            % Moves the robot to the specified position
            % position: [x, y, z] - Cartesian coordinates

            % Define the orientation (pen always perpendicular to the board)
            orientation = 0; % Orientation in radians (adjust if needed)

            % Compute joint angles using inverse kinematics
            ikSolver = obj.InverseKinematics();
            jointAngles = ikSolver.convertToJointAngles(position, orientation);

            % Command the robot to move to the computed joint angles
            motorIDs = [1, 2, 3, 4]; % Dynamixel motor IDs
            obj.Robot.robotMove(jointAngles, motorIDs);
        end
    end
end
