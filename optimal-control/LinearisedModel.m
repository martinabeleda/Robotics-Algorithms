function [A, B] = LinearisedModel()
global Ih mh rh Ib mb rb g

% Linearised Dynamics
M = [mb+Ib/rb^2+mh -mh*rh
     -mh*rh mh*rh^2+Ih];
C = zeros(2,2);
G = [0 0
     0 -mh*rh*g];
R = [1 rb]';
 
% State Space Model
a = [zeros(2,2) eye(2,2) zeros(2,1)
    -M\G -M\C M\R
    zeros(1,4) -2];
A = [a zeros(5)
     zeros(5) a];
b = [zeros(4,1)
     2];
B = [b zeros(5,1)
    zeros(5,1) b];

% Co = ctrb(A,B);
% fprintf('Rank of C = %d\n', rank(Co));
end

