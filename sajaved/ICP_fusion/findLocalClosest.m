function assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th)    

    %==== The output is the corresponding closest new points to each reference point ====
    assoc_pts = zeros(m, n, 3);

    %==== Create a cell to store the points within the local patch in a vectorized manner ====
    new_neighbors = cell(9,1);

    %==== Fill in new_neighbors{} with shifting points ====
    new_neighbors{2} = cat(1, new_pts(2:end, :, :), Inf(1, n, 3));
    new_neighbors{8} = cat(1, Inf(1, n, 3), new_pts(1:(end-1), :, :));
    new_neighbors{4} = cat(2, Inf(m, 1, 3), new_pts(:, 1:(end-1), :));   
    new_neighbors{6} = cat(2, new_pts(:, 2:end, :), Inf(m, 1, 3));
    new_neighbors{5} = new_pts;
    new_neighbors{3} = cat(2, new_neighbors{2}(:, 2:end, :), Inf(m, 1, 3));
    new_neighbors{1} = cat(2, Inf(m, 1, 3), new_neighbors{2}(:, 1:(end-1), :));
    new_neighbors{9} = cat(2, new_neighbors{8}(:, 2:end, :), Inf(m, 1, 3));
    new_neighbors{7} = cat(2, Inf(m, 1, 3), new_neighbors{8}(:, 1:(end-1), :));

    %==== Create a 3D matrix to store the square of distances between every reference point and it's corresponding local new points ====
    dist2 = zeros(m, n, 10);
    
    %==== All the last element in the 3rd dimension is set to be the square of the distance threshold ====
    dist2(:, :, 10) = d_th^2;
    
    %==== Fill in dist2[] with the square distance of a reference point to all the corresponding new points within the patch ====
    for i = 1:9
        dist2(:, :, i) = sum((ref_pts - new_neighbors{i}).^2, 3);
    end

    %==== Use dist2[] and new_neighbors{} to find the closest new points to each reference point, and set them into assoc_pts[] ==== 
    min_dist2 = min(dist2, [], 3);
    min_flag = (repmat(min_dist2, 1, 1, 10) == dist2);
    for i = 1:9
        assoc_pts(repmat(min_flag(:, :, i), 1, 1, 3)) = new_neighbors{i}(repmat(min_flag(:, :, i), 1, 1, 3));
    end 

end