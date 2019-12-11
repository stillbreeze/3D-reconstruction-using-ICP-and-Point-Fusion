function is_close = isInputCloseToProjPoints(input_points, proj_points, dist2_th)

    %==== Outputs a boolean matrix which represents if each corresponding point pairs are close enough given dist2_th ====
    diff_pts = proj_points - input_points;
    dist2_pts = sum(diff_pts.^2, 3);
    is_close = dist2_pts < dist2_th;

end
    