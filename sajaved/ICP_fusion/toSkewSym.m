function X = toSkewSym(x)
    
    %==== TODO: Output the 3-by-3 skew-symmetric matrix of the input 3-vector ====

    % Write your code here...
    X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end
