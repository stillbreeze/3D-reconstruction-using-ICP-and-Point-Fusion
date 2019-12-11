function alpha = getAlpha(points, sigma)
    
    %==== alpha[] is a matrix of the radial distance of the current depth measurement from the camera center ====
    alpha = exp(-sum(points.^2, 3)./(2*sigma^2));

end