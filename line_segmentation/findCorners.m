function corners = findCorners(vertices, tol)
% vectoriseVertices takes an array of xy coordinates where each row
%   of xy coordinates is adjacent to one another. This function returns
%   an array of xy coordinates where each row is a corner. Corners are 
%   characterised by vectors that are approximately perpendicular, which 
%   is validated if the determinate is close to 0. The tolerance at which 
%   corners are detected can be adjusted by the input tol.
    
    corners = [];

    for i = 2:(length(vertices) - 1)
        
        v1 = [(vertices(i-1,1)-vertices(i,1)), (vertices(i-1,2)-vertices(i,2))];
        v2 = [(vertices(i,1)-vertices(i+1,1)), (vertices(i,2)-vertices(i+1,2))];
        dotProd = dot(v1, v2);
        
        if abs(dotProd) < tol
           corners = [corners; [vertices(i, 1), vertices(i, 2)]]; 
        end
        
    end
    
end