function is_similar = isInputSimilarToProjNormals(input_normals, proj_normals, dot_th)

    %==== Outputs a boolean matrix which represents if each corresponding point normals are similar enough given dot_th ====    
    normal_dots = sum(input_normals.*proj_normals, 3);
    is_similar = normal_dots > dot_th;

end
    