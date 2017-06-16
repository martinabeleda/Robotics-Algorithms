function vertices = lineSegWrapper(x, y, thresh)
% lineSegWrapper is used to wrap the lineSegmentation function
%   so that the start and end points are added to the 
%   vertices dataset

    vertices = lineSegmentation(x, y, thresh);
    vertices = [[x(1), y(1)]; vertices; [x(end), y(end)]];

end

