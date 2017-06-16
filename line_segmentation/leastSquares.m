function [a, b] = leastSquares(x, y)
% leastSquares takes a set of XY coordinates and outputs the 
%   coefficients of fitted line in the form y = ax + b

    A = [mean(x.^2), mean(x);
         mean(x),    1];
    B = [mean(x.*y);
         mean(y)];
    X = A^-1*B; 

    a = X(1,1);
    b = X(2,1);
  
end