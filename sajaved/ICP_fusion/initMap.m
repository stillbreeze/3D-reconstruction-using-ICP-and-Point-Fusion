function fusion_map = initMap(pointcloud, normals, h, w, sigma)
    
    %==== Initialize the point-based fusion map ====
    map_pointcloud = pointCloud(reshape(pointcloud.Location, h*w, 3), 'Color', reshape(pointcloud.Color, h*w, 3));
    map_normals = reshape(normals, h*w, 3);
    map_ccounts = exp(-sum(map_pointcloud.Location.^2, 2)./(2*sigma^2));
    map_times = zeros(h*w, 1);

    %==== Output the initial point-based fusion map in a struct ====
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
       
end