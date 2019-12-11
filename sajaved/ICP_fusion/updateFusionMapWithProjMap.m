function fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag)

    %==== TODO: Merge the updated map with the remaining part of the old fusion map ====

    % Write your code here...

    new_pts = reshape(updated_map.points,[h*w,3]);
    new_normals = reshape(updated_map.normals,[h*w,3]);
    new_colors = reshape(updated_map.colors,[h*w,3]);
    new_ccounts = reshape(updated_map.ccounts,[h*w,1]);
    new_times = reshape(updated_map.times,[h*w,1]);
    
    unchanged_pts = fusion_map.pointcloud.Location(~proj_flag,:);
    map_points = [new_pts; unchanged_pts];

    unchanged_normals = fusion_map.normals(~proj_flag,:);
    map_normals = [new_normals; unchanged_normals];
    
    unchanged_colors = fusion_map.pointcloud.Color(~proj_flag,:);
    map_colors = [new_colors; unchanged_colors];
    
    unchanged_ccounts = fusion_map.ccounts(~proj_flag,:);
    map_ccounts = [new_ccounts; unchanged_ccounts];
    
    unchanged_times = fusion_map.times(~proj_flag,:);
    map_times = [new_times; unchanged_times];
    
    %==== Output the final point-based fusion map in a struct ====
    map_pointcloud = pointCloud(map_points, 'Color', map_colors);
    fusion_map = struct('pointcloud', map_pointcloud, 'normals', map_normals, 'ccounts', map_ccounts, 'times', map_times);
      
end
   