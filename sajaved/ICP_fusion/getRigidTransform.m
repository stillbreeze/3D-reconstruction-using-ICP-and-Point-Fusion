function [tform valid_pair_num error] = getRigidTransform(new_pointcloud, ref_pointcloud, ref_normals)
    
    %==== Initialize parameters ====
    iter_num = 6;
    d_th = 0.05;
    m = size(new_pointcloud.Location, 1);
    n = size(new_pointcloud.Location, 2);
    tform = affine3d(eye(4));
    
    %==== Main iteration loop ====
    for iter = 1:iter_num
        
        %==== Set variables ====
        new_pts = new_pointcloud.Location;
        ref_pts = ref_pointcloud.Location;
        
        %==== For each reference point, find the closest new point within a local patch of size 3-by-3 ====        
        %==== (Notice: assoc_pts[] has the same size and format as new_pts[] and ref_pts[]) ====
        %==== (Notice: assoc_pts[i, j, :] = [0 0 0] iff no point in new_pts[] matches to ref_pts[i, j, :]) ====
        assoc_pts = findLocalClosest(new_pts, ref_pts, m, n, d_th);
        
        %==== Set the sizes of matrix A[] and vertor b[] of normal equation: A'*A*x = A'*b ====
        A = zeros(m*n, 6);
        b = zeros(m*n, 1);
        
        %==== declare the number of point pairs that are used in this iteration ==== 
        valid_pair_num = 0;
        
        %==== TODO: Assign values to A[] and b[] ====
        %==== (Notice: the format of the desired 6-vector is: xi = [beta gamma alpha t_x t_y t_z]') ====

        % Write your code here...
        ref_pts = reshape(ref_pts,[m*n,3]);
        ref_normals = reshape(ref_normals,[m*n,3]);
        assoc_pts = reshape(assoc_pts,[m*n,3]);
        
        mask = sum(assoc_pts, 2) ~= 0;
        valid_pair_num = sum(mask);
        valid_points = reshape(find(mask), [1,valid_pair_num]);
        for i = valid_points
            G_i = [toSkewSym(assoc_pts(i,:)) eye(3,3)];
            A(i,:) = ref_normals(i,:) * G_i;
            diff = ref_pts(i,:) - assoc_pts(i,:);
            b(i,:) = ref_normals(i,:) * diff';
        end
        
        %==== TODO: Solve for the 6-vector xi[] of rigid body transformation ====

        % Write your code here...
        xi = A\b;

        %==== Coerce xi[] back into SE(3) ====
        %==== (Notice: tmp_tform[] is defined in the format of right-multiplication) ====
        R = toSkewSym(xi(1:3)) + eye(3);
        [U,~,V] = svd(R);
        R = U*V';
        T = [R [0 ; 0 ; 0] ; [xi(4:6)' 1]];
        tmp_tform = affine3d(T);
        
        %==== Updates the transformation and the pointcloud for the next iteration ====
        %==== (uses affine3d() and pctransform() functions) ====
        %==== (note the format of tform[] and the affine3d() function) ====
        tform = affine3d(tmp_tform.T*tform.T);
        if iter ~= iter_num
            new_pointcloud = pctransform(new_pointcloud, tmp_tform);
        end
        
    end
    
    %==== Find RMS error of point-plane registration ====
    error = sqrt(sum((A*xi - b).^2)/valid_pair_num);
end
        
