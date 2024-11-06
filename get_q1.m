function [q1] = get_q1(o03)
    x03 = o03(1);
    y03 = o03(2);
    q1 = atan2(y03,x03);
end