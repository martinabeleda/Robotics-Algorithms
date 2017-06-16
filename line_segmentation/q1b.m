% MTRX5700 - Assignment 2
% Martin Abeleda
% Q1 b
% Implement a function to compute the least square minimization for
% fitting a line to a list of scan points.

x = 1:10;
rng default;  % For reproducibility
figure('Color',[1 1 1]);

y = x + randn(1,10);
scatter(x,y,25,'b','*')
hold on
    
[a, b] = leastSquares(x, y);

plot(a*x + b, 'b');
title('Least Squares Minimisation');
ylabel('x');
xlabel('y');