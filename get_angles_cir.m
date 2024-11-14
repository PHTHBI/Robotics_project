function [q_cir] = get_angles_cir()
    num_points = 36;  
    [circle_points, x04_vector] = get_cir_cor();
    cir_angles = zeros(4,num_points + 1);
        
    for j = 1:(num_points + 1)
        cir_angles(:,j) = get_angles(circle_points(:,j), x04_vector(:,j));
    end
    q_cir = cir_angles;
end