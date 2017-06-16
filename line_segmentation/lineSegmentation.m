function  vertices = lineSegmentation(x, y, thresh)
% lineSegmentation takes a set of XY coordinates and 
%   outputs vertices that segment the lines based on
%   the user input threshold
    
    D = zeros(1, length(x));

    % Find the maximum perpendicular distance between the line and the points
    for i = 1:length(x)
        D(i) = perpDist(x(1), y(1), x(end), y(end), x(i), y(i));
    end

    % Find index of the maximum point
    [new_max, ind] = max(D);

    % If the max is > thresh, segment at this vertex
    if  new_max > thresh 
        vertices1 = lineSegmentation(x(1:ind), y(1:ind), thresh);
        vertices2 = lineSegmentation(x(ind:end), y(ind:end), thresh); 
        vertices = [vertices1; [x(ind), y(ind)]; vertices2];
    else  % Not a corner so return nothing
        vertices = [];
    end

end