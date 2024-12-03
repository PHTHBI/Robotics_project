classdef MarkerDrawer
    properties
        InverseKinematics % Instance of the InverseKinematics class
        Robot             % Instance of the RobotController class
        MarkerOffset      % Offset of the marker from the robot’s end effector
    end

    methods
        function obj = MarkerDrawer(robot)
            % Constructor: Initialize with robot instance and marker offset
            obj.Robot = robot;
            obj.InverseKinematics = InverseKinematics();
            obj.MarkerOffset = [30, 30, 0]; % [mm] Offset for the marker
        end

        function drawX(obj, cellCenter)
            % Draws an "X" at the given cell center
            % cellCenter: [x, y] position on the board in mm
            
            % Define line endpoints for the "X" (diagonals within the cell)
            lineOffset = 15; % Half the size of the cell's diagonal for the "X"
            line1Start = [cellCenter(1) - lineOffset, cellCenter(2) - lineOffset];
            line1End = [cellCenter(1) + lineOffset, cellCenter(2) + lineOffset];
            line2Start = [cellCenter(1) - lineOffset, cellCenter(2) + lineOffset];
            line2End = [cellCenter(1) + lineOffset, cellCenter(2) - lineOffset];

            % Draw the first line of the "X"
            obj.moveToPoint(line1Start, -0.5); % Move to starting point
            obj.moveToPoint(line1End, -0.5);   % Draw to ending point
            
            % Lift marker and move to the second line's start
            obj.moveToPoint(line1End, 5);      % Lift marker
            obj.moveToPoint(line2Start, 5);    % Move above starting point
            
            % Draw the second line of the "X"
            obj.moveToPoint(line2Start, -0.5); % Move to starting point
            obj.moveToPoint(line2End, -0.5);   % Draw to ending point
            
            % Lift marker back to clearance height
            obj.moveToPoint(line2End, 5);
        end

        function drawO(obj, cellCenter)
            % Draws an "O" at the given cell center
            % cellCenter: [x, y] position on the board in mm
            
            % Define circle properties
            radius = 15; % Radius of the circle
            numPoints = 36; % Number of points for smoothness
            theta = linspace(0, 2 * pi, numPoints);

            % Compute circle points
            circlePoints = [cellCenter(1) + radius * cos(theta)', ...
                            cellCenter(2) + radius * sin(theta)'];

            % Move to the first point on the circle
            obj.moveToPoint(circlePoints(1, :), 5); % Move above start
            obj.moveToPoint(circlePoints(1, :), -0.5); % Move to start
            
            % Draw the circle
            for i = 2:numPoints
                obj.moveToPoint(circlePoints(i, :), -0.5); % Draw to next point
            end

            % Close the circle by connecting the last and first points
            obj.moveToPoint(circlePoints(1, :), -0.5);

            % Lift marker back to clearance height
            obj.moveToPoint(circlePoints(1, :), 5);
        end

        function moveToPoint(obj, point, zOffset)
            % Moves the robot to a specific 3D point
            % point: [x, y] 2D position in mm
            % zOffset: z-coordinate offset (e.g., clearance height or drawing height)
            
            % Combine the 2D point with the zOffset to get the full 3D position
            targetPosition = [point(1), point(2), zOffset];

            % Use inverse kinematics to calculate joint angles
            jointAngles = obj.InverseKinematics.convertToJointAngles(targetPosition, 0); % Assuming 0 orientation

            % Move the robot to the calculated joint angles
            obj.Robot.robotMove(jointAngles); % Adjust as per your robot's move method
        end
    end
end
