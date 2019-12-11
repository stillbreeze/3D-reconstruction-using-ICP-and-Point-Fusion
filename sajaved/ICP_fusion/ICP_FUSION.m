%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #4                         %
%  ICP and Point-based Fusion            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%==== Only load data when needed ====
if ~exist('seq_param_loaded')
    clear
    clc
    tic
    fprintf('Loading data...');
    load ../data/seq;
    load ../data/cam_param; %cam_param = [fx fy cx cy], h, w
    fprintf('Time spent on loading: %.2f sec \n', toc);
    seq_param_loaded = true;
end

%==== TEST: Debug ICP or point-based fusion (0: false, 1: true)====
is_debug_icp = 0;
is_debug_fusion = 0;
is_eval = 1;

%==== Set start time ====
tic;

%==== Start recording new summary file ====
delete summary.txt;
diary off;
diary on;

%==== Set parameters ====
sigma = 0.6; %the variance of the Gaussian weight used in fusion
T = size(seq, 1);
grid_size = 0.005;
positions = zeros(T, 3);

%==== Set downsampling ratio for ICP ====
ds_ratio = 16;

%==== Start main loop ====
for t = 1:T
    
    %==== Display frame# ====
    fprintf('Frame#: %d', t);
    
    %==== Get input data from seq{} ====
    pointcloud = seq{t, 1};
    normals = seq{t, 2};
    
    %==== Downsample input data ====
    [ds_pointcloud, ds_normals] = downsampleData(pointcloud, normals, ds_ratio);
     
    %==== If this is the very first frame ====
    if t == 1

        %==== Initialize the fusion map ====            
        fusion_map = initMap(pointcloud, normals, h, w, sigma);
         
        %==== Initialize the first 4-by-4 pose matrix to identity ====
        current_pose = affine3d(eye(4));
        
        %==== Initialize the reference pointcloud and normals ====
        ref_pointcloud = ds_pointcloud;
        ref_normals = ds_normals;
        fprintf(' (ICP and Point-based fusion initialized)\n');
        
    else
        
        %==== Transform the input pointcloud by the last transformation for better registration ====
        %==== (Notice: because of the format of pctransform(), last_pose[] is defined in the format of right-multiplication) ====
        new_pointcloud = pctransform(ds_pointcloud, last_pose);
        
        %==== 1) Apply point-to-plane ICP to register the input pointcloud to the reference pointcloud ====
        if is_debug_fusion == 0
            [tform inliers error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals);
            fprintf(' (ICP final iteration: inliers = %d, RMSE = %d)', inliers, error);
        
        %==== DEBUG 1): A built-in registration function for debugging point-based fusion only ====
        else
            tform = pcregrigid(new_pointcloud, ref_pointcloud, 'Metric', 'pointToPlane', 'Extrapolate', true);
        end
        fprintf('\n');
        
        %==== Update the transformation ====
        %==== (Notice: because of the format of affine3d(), the transformation is defined in the format of right-multiplication) ====
        current_pose = affine3d(tform.T*last_pose.T);
        
        %==== Record the position for later plot ====
        positions(t, :) = current_pose.T(4, 1:3)';
        
        %==== Set the input data for fusion ====
        input_data = struct('pointcloud', pointcloud, 'normals', normals);
        
        %==== 2) Apply point-based fusion to get a global fusion map and the next reference data ====
        if is_debug_icp == 0
            [fusion_map, next_ref_data] = pointBasedFusion(fusion_map, input_data, current_pose, cam_param, sigma, ds_ratio, t);      
            ref_pointcloud = next_ref_data.pointcloud;
            ref_normals = next_ref_data.normals;
        
        %==== DEBUG 2): A built-in pointcloud merging function for debugging ICP registration only
        else
            trans_pointcloud = pctransform(pointcloud, current_pose);
            fusion_map.pointcloud = pcmerge(fusion_map.pointcloud, trans_pointcloud, grid_size);
            ref_pointcloud = pctransform(ds_pointcloud, current_pose);
            ref_normals = nvRotate(ds_normals, current_pose);
        end
    end
    
    %==== Store the current transformation for the next frame ====
    last_pose = current_pose;
end

%==== Plot output 3D fusion map ====
plotTrajAndMap(positions, fusion_map);

%==== EVAL: Visualize normals, ccounts, and times ====
plotEvalMaps(fusion_map, is_eval);

%==== Display final results ====
format short;
fprintf('______________________________________________________________________\n');
fprintf('Number of points in the final model = %d \n', fusion_map.pointcloud.Count);
fprintf('Compression ratio = %.2f %% \n', fusion_map.pointcloud.Count/(h*w*T)*100);
fprintf('Total time spent = %.2f sec \n', toc);

%==== End recording summary file ====
diary summary.txt;
