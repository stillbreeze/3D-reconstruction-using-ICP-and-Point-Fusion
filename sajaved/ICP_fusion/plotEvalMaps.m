function plotEvalMaps(fusion_map, is_eval)
    if is_eval == 1
        num = fusion_map.pointcloud.Count;
        pts = fusion_map.pointcloud.Location;
        nvs = fusion_map.normals;
        sparse = [1:800:num];
        scale = 1;
        %==== Plot normals ====
        figure(2);
        hold on;
        title('normal vectors');
        rand_idx = randperm(num);
        rand_pts = pts(rand_idx, :);
        rand_nvs = nvs(rand_idx, :);
        nv_pointcloud = pointCloud(rand_pts, 'Color', im2uint8(rand_nvs/5+0.8));
        showPointCloud(nv_pointcloud);
        quiver3(rand_pts(sparse, 1), rand_pts(sparse, 2), rand_pts(sparse, 3), rand_nvs(sparse,1), rand_nvs(sparse,2), rand_nvs(sparse,3), scale);
        %==== Plot ccounts ====
        figure(3);
        hold on;
        title('confidence counts');
        cc_color = repmat(log(fusion_map.ccounts), 1, 2);
        %cc_color = log(fusion_map.ccounts);
        cc_min = min(min(cc_color));
        cc_max = max(max(cc_color));
        cc_color = cat(2, zeros(fusion_map.pointcloud.Count, 1), (cc_color - cc_min)/(cc_max - cc_min));
        cc_pointcloud = pointCloud(pts, 'Color', im2uint8(cc_color));
        showPointCloud(cc_pointcloud);
        %==== Plot times ====
        figure(4);
        hold on;
        title('time stamps');
        t_color = repmat(fusion_map.times, 1, 2);
        t_color = cat(2, t_color, ones(fusion_map.pointcloud.Count, 1));
        t_pointcloud = pointCloud(pts, 'Color', im2uint8(t_color/max(max(t_color))));
        showPointCloud(t_pointcloud);
        
    end
end
