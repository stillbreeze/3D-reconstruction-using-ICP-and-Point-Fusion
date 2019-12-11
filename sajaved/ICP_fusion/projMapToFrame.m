function [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param)
    
    %==== Set parameters ====
    fx = cam_param(1);
    fy = cam_param(2);
    cx = cam_param(3);
    cy = cam_param(4);

    %==== TODO: Project all terms in the fusion map based on other input parameters ====
    %==== (Hint 1: Only project the points in front of the camera) ====
    %==== (Hint 2: Calculate all the projected indices together and use them in vectorization) ====
    %==== (Hint 3: Discard the indices that exceed the frame boundaries) ====

    % Write your code here...

    pts = fusion_map.pointcloud.Location;
    transformed_pointcloud = pctransform(fusion_map.pointcloud, invert(tform));
    transformed_pts = transformed_pointcloud.Location;
    K = [fx 0 cx; 0 fy cy; 0 0 1];
    projected_pts = (K*transformed_pts')';
    projected_pts = round(projected_pts ./ projected_pts(:,3));
    
    negz_mask = transformed_pts(:,3) > 0;
    bound_mask = projected_pts(:,1) >= 1 & projected_pts(:,2) >= 1 & ...
                 projected_pts(:,1) <= w & projected_pts(:,2) <= h;
    proj_flag = negz_mask & bound_mask;


    valid_mask_pts = find(proj_flag);
    valid_projected_pts = projected_pts(valid_mask_pts, :);
    x = valid_projected_pts(:,1);
    y = valid_projected_pts(:,2);
    
    
    
%     proj_points = zeros(h,w,3);
%     proj_colors = zeros(h,w,3);
%     proj_normals = zeros(h,w,3);
%     proj_ccounts = zeros(h,w,1);
%     proj_times = zeros(h,w,1);
%     colors = fusion_map.pointcloud.Color;
%     normals = fusion_map.normals;
%     ccounts = fusion_map.ccounts;
%     times = fusion_map.times;    
%     for i=1:size(valid_mask_pts)
%         proj_points(y(i), x(i), :) = pts(valid_mask_pts(i), :);
%         proj_colors(y(i), x(i), :) = colors(valid_mask_pts(i), :);
%         proj_normals(y(i), x(i), :) = normals(valid_mask_pts(i), :);
%         proj_ccounts(y(i), x(i)) = ccounts(valid_mask_pts(i));
%         proj_times(y(i), x(i)) = times(valid_mask_pts(i));
%     end
    


    proj_points = zeros(h*w,3);
    proj_points(sub2ind([h w], y, x), :) = pts(valid_mask_pts, :);
    proj_points = reshape(proj_points, h, w, 3);
    
    color = fusion_map.pointcloud.Color;
    proj_colors = zeros(h*w,3);
    proj_colors(sub2ind([h w], y, x), :) = color(valid_mask_pts, :);
    proj_colors = reshape(proj_colors, h, w, 3);
    
    normals = fusion_map.normals;
    proj_normals = zeros(h*w,3);
    proj_normals(sub2ind([h w], y, x), :) = normals(valid_mask_pts, :);
    proj_normals = reshape(proj_normals, h, w, 3);
    
    ccounts = fusion_map.ccounts;
    proj_ccounts = zeros(h*w,1);
    proj_ccounts(sub2ind([h w], y, x)) = ccounts(valid_mask_pts);
    proj_ccounts = reshape(proj_ccounts, h, w);
    
    times = fusion_map.times;
    proj_times = zeros(w*h,1);
    proj_times(sub2ind([h w], y, x)) = times(valid_mask_pts);
    proj_times = reshape(proj_times, h, w);
    
    %==== Output the projected map in a struct ====
    %==== (Notice: proj_points[], proj_colors[], and proj_normals[] are all 3D matrices with size h*w*3) ====
    %==== (Notice: proj_ccounts[] and proj_times[] are both 2D matrices with size h*w) ====
    proj_map = struct('points', proj_points, 'colors', proj_colors, 'normals', proj_normals, 'ccounts', proj_ccounts, 'times', proj_times);
        
end
