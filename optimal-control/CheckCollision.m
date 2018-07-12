function hit = CheckCollision(x1,x2,As,cs)
%Students to add code that returns:
% hit = 1 if the line from x1 to x2 hits the ellipse (x-c)'A(x-c)<=1.
% hit = 0 otherwise (i.e. path is safe)
global BufferDist

% break the line into a vector of points
coef = polyfit([x1(1), x2(1)], [x1(2), x2(2)], 1);
x_points = linspace(x1(1),x2(1));
y_points = coef(1)*x_points+coef(2);
x = [x_points; y_points];

% test collision with each obstacle
for k = 1:length(As)
    % decompose A matrix and add buffer zone
    [U, D, V] = svd(As{k});
    a = 1/sqrt(D(1,1))+BufferDist;
    b = 1/sqrt(D(2,2))+BufferDist;
    D(1,1) = 1/a^2;
    D(2,2) = 1/b^2;
    A = U*D*V;

    % scan along the line 
    for i = 1:length(x_points)
        % point lies inside the ellipse
        if (x(:,i)-cs{k})'*A*(x(:,i)-cs{k}) <= 1 
            hit = 1; 
            return
        end
    end
end
% no hits
hit = 0;
end