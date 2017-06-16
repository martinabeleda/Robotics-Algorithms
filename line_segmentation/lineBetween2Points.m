function [a, b] = lineBetween2Points(x1, x2, y1, y2)
%lineBetween2Points This function returns the gradient and y intercept
%   of a line between two points (x1, y1) and (x2, y2) in the form
%   y = ax + b

    a = (y2 - y1)/(x2 - x1);
    b = -a*x1 + y1;
end

