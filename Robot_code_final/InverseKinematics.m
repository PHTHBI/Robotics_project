classdef InverseKinematics
    properties (Constant)
        % DH Parameters
        a1 = 50; % [mm]
        a2 = 93; % [mm]
        a3 = 93; % [mm]
        a4 = 75; % [mm] Updated to fit with pen attached
    end

    methods
        function jointAnglesRad = convertToJointAngles(obj, position, orientation, theta3Option)
            % Converts Cartesian coordinates to joint angles
            % position: [x, y, z] - Cartesian position of the end effector
            % orientation: stylus orientation in radians
            % theta3Option: 1 or 2 for solving the third joint angle

            if nargin < 4
                theta3Option = 1; % Default to "elbow down"
            end

            % Extract x, y, z from the position vector
            x_s = position(1);
            y_s = position(2);
            z_s = position(3) - obj.a1; % Adjust z by the height of the base

            % Calculate x_hat (distance in x-y plane)
            x_hat = sqrt(x_s^2 + y_s^2);

            % Calculate the position of joint 3 (p03) in the x-z plane
            p03_x_hat = x_hat - obj.a4 * cos(orientation);
            p03_z = z_s - obj.a4 * sin(orientation);

            % Distance between joint 1 and joint 3
            d = sqrt(p03_x_hat^2 + p03_z^2);

            % Joint 1 angle (q1)
            q1 = atan2(y_s, x_s);

            % Joint 3 angle (q3) - two options
            c3 = (-obj.a2^2 - obj.a3^2 + d^2) / (2 * obj.a2 * obj.a3);
            if theta3Option == 1
                q3 = atan2(sqrt(1 - c3^2), c3); % "Elbow down"
            elseif theta3Option == 2
                q3 = atan2(-sqrt(1 - c3^2), c3); % "Elbow up"
            else
                error('Invalid theta3Option. Must be 1 or 2.');
            end

            % Joint 2 angle (q2)
            q2 = -atan2(p03_x_hat, p03_z) - atan2(obj.a3 * sin(q3), obj.a2 + obj.a3 * cos(q3)) + pi / 2;

            % Joint 4 angle (q4)
            q4 = orientation - q3 - q2;

            % Combine all angles into a vector
            jointAnglesRad = [q1, q2, q3, q4];

            % Convert angles to deg
            %jointAngles = rad2deg(jointAnglesRad);

            
        end

        function cartesianPosition = computeCartesian(obj, jointAngles)
            % Computes Cartesian coordinates from joint angles
            % Inputs:
            % - jointAngles: [q1, q2, q3, q4] angles of the robot joints
            % Outputs:
            % - cartesianPosition: [x, y, z] position of the end-effector

            q1 = jointAngles(1);
            q2 = jointAngles(2);
            q3 = jointAngles(3);
            q4 = jointAngles(4);

            % Compute transformations using forward kinematics
            % Position of each joint
            x1 = 0; y1 = 0; z1 = obj.a1; % Base to Joint 1
            x2 = obj.a2 * cos(q2) * cos(q1);
            y2 = obj.a2 * cos(q2) * sin(q1);
            z2 = obj.a1 + obj.a2 * sin(q2);

            x3 = x2 + obj.a3 * cos(q2 + q3) * cos(q1);
            y3 = y2 + obj.a3 * cos(q2 + q3) * sin(q1);
            z3 = z2 + obj.a3 * sin(q2 + q3);

            x4 = x3 + obj.a4 * cos(q1) * cos(q2 + q3 + q4);
            y4 = y3 + obj.a4 * sin(q1) * cos(q2 + q3 + q4);
            z4 = z3 + obj.a4 * sin(q2 + q3 + q4);

            % End-effector position
            cartesianPosition = [x4, y4, z4];
        end

        function validateAngles(obj, jointAngles)
            % Validates that the computed joint angles are within the robot's limits
            % Inputs:
            % - jointAngles: [q1, q2, q3, q4] angles to be validated

            % Define joint limits (example limits, adjust for your robot)
            jointLimits = [
                0, 300;        % Joint 1
                0, 300;        % Joint 2
                40, 200;        % Joint 3
                40, 200         % Joint 4
            ];

            % Check each joint angle
            for i = 1:length(jointAngles)
                if jointAngles(i) < jointLimits(i, 1) || jointAngles(i) > jointLimits(i, 2)
                    error('Joint %d angle out of bounds: %.2f', i, jointAngles(i));
                end
            end
        end
    end
end
