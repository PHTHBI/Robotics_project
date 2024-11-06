function [q] = get_angles(o04, x04)
    o03 = geto03(o04,x04)
    q1 = get_q1(o03);
    q2 = get_q2(o03);
    q3 = get_q3(o03);
    q4 = get_q4(q2,q3,x04);

    q = zeros(1,4);
    q(1) = q1;
    q(2) = q2;
    q(3) = q3;
    q(4) = q4;
    q = q;

end