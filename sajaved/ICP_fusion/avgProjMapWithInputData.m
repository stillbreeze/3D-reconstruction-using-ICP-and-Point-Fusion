function updated_map = avgProjMapWithInputData(proj_map, input_data, alpha, h, w, is_use, t)

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    input_colors = input_data.pointcloud.Color;
    input_normals = input_data.normals;
    proj_points = proj_map.points;
    proj_colors = proj_map.colors;
    proj_normals = proj_map.normals;
    proj_ccounts = proj_map.ccounts;
    proj_times = proj_map.times;

    %==== TODO: Update all the terms in the projected map using the input data ====
    %==== (Hint: apply is_use[] as a mask in vectorization) ====

    % Write your code here...
    updated_points = (proj_ccounts .* proj_points + alpha .* input_points .* is_use) ./ (proj_ccounts + alpha .* is_use);
    updated_normals = (proj_ccounts .* proj_normals + alpha .* input_normals .* is_use) ./ (proj_ccounts + alpha .* is_use);
    updated_colors = (proj_ccounts .* double(proj_colors) + alpha .* double(input_colors) .* is_use) ./ (proj_ccounts + alpha .* is_use);
    updated_ccounts = proj_ccounts + alpha .* is_use;
    updated_times = proj_times;
    updated_times(is_use) = t;
    
    %==== Output the updated projected map in a struct ====
    updated_map = struct('points', updated_points, 'colors', updated_colors, 'normals', updated_normals, 'ccounts', updated_ccounts, 'times', updated_times);
        
end