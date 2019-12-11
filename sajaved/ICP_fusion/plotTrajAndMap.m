function plotTrajAndMap(positions, fusion_map)
    
    close all;
    cloud = fusion_map.pointcloud;
    %==== Complete output ====
    if size(fusion_map.normals) == size(fusion_map.pointcloud.Location)
        figure('Position', [100, 100, 800, 400]);
        title('Output fusion map with color and lighting');
        
        %==== Plot pointcloud with color ====
        subplot(1,2,1);
        hold on;
        axis([cloud.XLimits cloud.YLimits 0 cloud.ZLimits(2)]);
        showPointCloud(cloud);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'r-', 'LineWidth', 2);
        plot3(positions(1, 1), positions(1, 2), positions(1, 3), 'k*');
        plot3(positions(end, 1), positions(end, 2), positions(end, 3), 'b*');   
        view([0 -50]);

        %==== Plot pointcloud with lighting ====
        subplot(1,2,2);
        hold on;
        axis([cloud.XLimits cloud.YLimits 0 cloud.ZLimits(2)]);
        nv_pointcloud = pointCloud(cloud.Location, 'Color', im2uint8(repmat(fusion_map.normals(:, 1) + fusion_map.normals(:, 3), 1, 3)/2/3 + 0.66));
        showPointCloud(nv_pointcloud);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'r-', 'LineWidth', 2);
        plot3(positions(1, 1), positions(1, 2), positions(1, 3), 'k*');
        plot3(positions(end, 1), positions(end, 2), positions(end, 3), 'b*'); 
        view([0 -50]);
        
    %==== Debugging output ====
    else
        figure(1);
        hold on;
        axis([cloud.XLimits cloud.YLimits 0 cloud.ZLimits(2)]);
        showPointCloud(cloud);
        plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'r-', 'LineWidth', 2);
        plot3(positions(1, 1), positions(1, 2), positions(1, 3), 'k*');
        plot3(positions(end, 1), positions(end, 2), positions(end, 3), 'b*');   
        view([0 -50]);
    end
end