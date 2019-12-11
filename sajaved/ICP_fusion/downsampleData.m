function [ds_pointcloud, ds_normals] = downsampleData(pointcloud, normals, ds_ratio)
    
    %==== Downsample both pointcloud and normals ====
    ds_points = pointcloud.Location(1:ds_ratio:end, 1:ds_ratio:end, :);
    ds_colors = pointcloud.Color(1:ds_ratio:end, 1:ds_ratio:end, :);
    ds_pointcloud = pointCloud(ds_points, 'Color', ds_colors);
    ds_normals = normals(1:ds_ratio:end, 1:ds_ratio:end, :);
    
end
    