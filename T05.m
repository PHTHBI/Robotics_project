function [T05] = T05(q1, q2, q3, q4); 
    %Setup 
    LINK1 = 50;
    LINK2 = 93;
    LINK3 = LINK2;
    LINK4 = 50;
    CAMERA_Y = 45;
    CAMERA_X = 35;
    CAMERA_X_TO_4 = -15;
    LINK5 = sqrt(CAMERA_X^2 + CAMERA_Y^2); % from o3 to camera
    CAMERA_STYLUS_ANGLE = atan(9/7);
    
    %T04 from angle q1-4

    %T01
    %q1 = rotation around z-axis in radians, 
    Rz1 = [cos(q1) -sin(q1) 0 0; 
        sin(q1) cos(q1) 0 0; 
        0 0 1 0
        0 0 0 1]; %Rotation around z-axis by q2 angle

    TransZ1 = [1 0 0 0
            0 1 0 0
            0 0 1 LINK1
            0 0 0 1]; %Translation a1 along z-axis

    Rx1 = [1 0 0 0; 
        0 cos(pi/2) -sin(pi/2) 0; 
        0 sin(pi/2) cos(pi/2) 0
        0 0 0 1];

    T01 = Rz1 * TransZ1 * Rx1;

    %T12
    %q2 = rotation around z-axis in radians.
    
    Rz2 = [cos(q2+pi/2) -sin(q2+pi/2) 0 0; 
        sin(q2+pi/2) cos(q2+pi/2) 0 0; 
        0 0 1 0
        0 0 0 1]; %Rotation around z-axis by q2 angle plus an additional pi/s to allign x-axis

    TransX2 = [1 0 0 LINK2
        0 1 0 0
        0 0 1 0
        0 0 0 1]; %Translation a2 along x-axis

    T12 = Rz2 * TransX2;

    %T23
    %q3 = rotation around z-axis in radians.

    Rz3 = [cos(q3) -sin(q3) 0 0; 
        sin(q3) cos(q3) 0 0; 
        0 0 1 0
        0 0 0 1]; %Rotation around z-axis by theta1 angle

    TransX3 = [1 0 0 LINK3
        0 1 0 0
        0 0 1 0
        0 0 0 1]; %Translation a2 along x-axis


    T23 = Rz3 * TransX3;

    %T34
    %q4 = rotation around z-axis in radians.

    Rz4 = [cos(q4) -sin(q4) 0 0; 
        sin(q4) cos(q4) 0 0; 
        0 0 1 0
        0 0 0 1]; %Rotation around z-axis by theta1 angle

    TransX4 = [1 0 0 LINK4
        0 1 0 0
        0 0 1 0
        0 0 0 1]; %Translation a2 along x-axis


    T34 = Rz4 * TransX4;

    %T45
    %q4 = rotation around z-axis in radians.

    T45 = [1, 0, 0, CAMERA_X_TO_4;
           1, 1, 0, 0;
           0, 0, 1, CAMERA_Y;
           0, 0, 0, 1];

    T05 = T01*T12*T23*T34*T45;

end

    

    