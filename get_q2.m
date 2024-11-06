function [q2] = get_q2(o03)
    LINK1 = 50;
    LINK2 = 93;
    LINK3 = LINK2;
    LINK4 = 50;
    CAMERA_Y = 45;
    CAMERA_X = 35;
    CAMERA_X_TO_4 = -15;
    LINK5 = sqrt(CAMERA_X^2 + CAMERA_Y^2); % from o3 to camera
    CAMERA_STYLUS_ANGLE = atan(9/7);
    
    x03 = o03(1);
    y03 = o03(2);
    z03 = o03(3);

    r = sqrt(x03^2 + y03^2);
    s = z03 - LINK1;
    c3 = (r^2 + s^2 - LINK2^2 - LINK3^2) / (2*LINK2*LINK3);
    s3 = sqrt(1-c3^2);

    q2 = atan2(r,s) - atan2(LINK2+LINK3*c3, LINK3*s3)
end