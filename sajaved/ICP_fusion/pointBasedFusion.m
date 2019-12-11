function [fusion_map, next_ref_data] = pointBasedFusion(fusion_map, input_data, tform, cam_param, sigma, ds_ratio, t)
    
    %==== TEST: Set parameters (square of point distance threshold) ====
    dist_th = 0.05; %5cm
    dist2_th = dist_th^2;
    dot_th = cos(20*pi/180); %20degrees

    %==== Set variables ====
    input_points = input_data.pointcloud.Location;
    h = size(input_points, 1);
    w = size(input_points, 2);
    
    %==== Transform the input data for fusion ====
    trans_pointcloud = pctransform(input_data.pointcloud, tform); 
    trans_points = trans_pointcloud.Location;
    trans_normals = nvRotate(input_data.normals, tform);
    trans_data = struct('pointcloud', trans_pointcloud, 'normals', trans_normals);
    
    %==== Project the global fusion map onto the input image frame ==== 
    [proj_map, proj_flag] = projMapToFrame(fusion_map, h, w, tform, cam_param); 
    proj_points = proj_map.points;
    proj_normals = proj_map.normals;
    
    %==== Find if each projected map point and its corresponding input point are close in distance ====
    is_close = isInputCloseToProjPoints(trans_points, proj_points, dist2_th);
    
    %==== Find if each projected map point and its corresponding input point are similar in normal direction ====
    is_similar = isInputSimilarToProjNormals(trans_normals, proj_normals, dot_th);
    
    %==== Find if each input point is a newly observed point ====
    is_first = sum(proj_points.^2, 3) == zeros(h, w);
    
    %==== Determine if each input point should be added into the fusion map ====
    is_use = isUsableInputPoints(is_close, is_similar, is_first);
    
    %==== Calculate alpha[] based on the radial distance of each point to the camera center ==== 
    alpha = getAlpha(input_points, sigma);
    
    %==== Average the projected map with the input data ====
    updated_map = avgProjMapWithInputData(proj_map, trans_data, alpha, h, w, is_use, t);
    
    %==== Downsample the updated map to get the reference data for next ICP registraion ===
    next_ref_data = getNextRefData(updated_map, ds_ratio);
    
    %==== Merge the averaged projected map with the unchanged part of the original fusion map to get the final fusion map ====
    fusion_map = updateFusionMapWithProjMap(fusion_map, updated_map, h, w, proj_flag);
      
end