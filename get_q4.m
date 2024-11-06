function [q4] = get_q4(q2, q3, x04)
    LINK1 = 50;
    LINK2 = 93;
    LINK3 = LINK2;
    LINK4 = 50;
    CAMERA_Y = 45;
    CAMERA_X = 35;
    CAMERA_X_TO_4 = -15;
    LINK5 = sqrt(CAMERA_X^2 + CAMERA_Y^2); % from o3 to camera
    CAMERA_STYLUS_ANGLE = atan(9/7);

    z03 = x04(3);

    q4 = asin(z03) - q2 - q3;  
end