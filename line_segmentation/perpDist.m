function d = perpDist(x1, y1, x2, y2, u, v)
% perpDist computes the perpendicular distance (d) from a line
%   characterised by the start and end points (x1, y1) and (x2, y2) 
%   to a point (u,v)   

    r = u*(y1 - y2) + v*(x2 - x1) + y2*x1 - y1*x2;
    D = sqrt((x2-x1)^2+(y2-y1)^2);
    d = abs(r/D);

end
