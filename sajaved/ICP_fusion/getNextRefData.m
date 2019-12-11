function next_ref_data = getNextRefData(updated_map, ds_ratio)

    %==== Generates the downsampled pointcloud and normals ====
    %==== (Similar to the "Downsample input data" part in "ICP_FUSION.m") ====
    updated_points = updated_map.points;
    updated_colors = updated_map.colors;
    updated_normals = updated_map.normals;
    updated_pointcloud = pointCloud(updated_points, 'Color', updated_colors);  
    [next_ref_pointcloud, next_ref_normals] = downsampleData(updated_pointcloud, updated_normals, ds_ratio);
    
    %==== Output the next reference data in a struct ====
    next_ref_data = struct('pointcloud', next_ref_pointcloud, 'normals', next_ref_normals);   

end
    