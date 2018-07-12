function hit = CheckCollisionNode(x, As, cs)
%CheckCollisionNode - Returns 1 if point x collides with any of the
%obstacles defined by As and cs. Returns 0 if no collision
global BufferDist

for i = 1:length(As)
    [U, D, V] = svd(As{i});
    a = 1/sqrt(D(1,1))+BufferDist;
    b = 1/sqrt(D(2,2))+BufferDist;
    D(1,1) = 1/a^2;
    D(2,2) = 1/b^2;
    A = U*D*V;
    
    if (x-cs{i})'*A*(x-cs{i}) <= 1 
        hit = 1;    
        return
    end  
end
hit = 0;
end

