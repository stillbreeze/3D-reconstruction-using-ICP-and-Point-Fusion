function rot_normals = nvRotate(normals, tform)
    
    %==== Set parameters ====
    m = size(normals, 1);
    n = size(normals, 2);
    
    %==== Rotate all the normal vectors stored in the m-by-n-by-3 matrix: normals[] ====
    R = tform.T(1:3, 1:3);
    rot_normals = zeros(m, n, 3);
    rot_normals(:, :, 1) = normals(:, :, 1)*R(1, 1) + normals(:, :, 2)*R(2, 1) + normals(:, :, 3)*R(3, 1);  
    rot_normals(:, :, 2) = normals(:, :, 1)*R(1, 2) + normals(:, :, 2)*R(2, 2) + normals(:, :, 3)*R(3, 2);
    rot_normals(:, :, 3) = normals(:, :, 1)*R(1, 3) + normals(:, :, 2)*R(2, 3) + normals(:, :, 3)*R(3, 3);
    
end