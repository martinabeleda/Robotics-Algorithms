function params = ComputePath(As, cs, G, s, f, path_colour)
global conn dispPRM dispNodes

%%% Add start and finish nodes to the graph and link with neighbours
G = addnode(G, table(s(1), s(2), 'VariableNames', {'x', 'y'}));
G = addnode(G, table(f(1), f(2), 'VariableNames', {'x', 'y'}));
sIdx = size(G.Nodes,1)-1; tIdx = size(G.Nodes,1);
for i = (size(G.Nodes,1)-1):size(G.Nodes,1)
    x1 = G.Nodes{i, {'x', 'y'}};
    Idx = knnsearch(G.Nodes{:, {'x', 'y'}}, x1, 'K', conn+1);
    Idx = Idx(Idx~=i); % remove current node
    for j = 1:conn
        x2 = G.Nodes{Idx(j), {'x', 'y'}};
        if and(~CheckCollision(x1', x2', As, cs), findedge(G, i, Idx(j)) == 0)  
            G = addedge(G, i, Idx(j), norm(x1-x2));
            if dispPRM
                plot([x1(1) x2(1)], [x1(2) x2(2)], 'b-');
            end
        end
    end
end

%%% Path Planning Phase - run Dijkstra's algorithm from start to all nodes
% first create adjacency matrix representation
nn = numnodes(G);
[ss,t] = findedge(G);
A = full(sparse(ss,t,G.Edges.Weight,nn,nn));
A = A + A.' - diag(diag(A));
T = Dijkstra(A, sIdx);
waypoints = OptimalPath(T, G, tIdx, true, path_colour);

%%% Output Parameters
params = cell(1,2);     
params{1,1} = waypoints;    % Co-ordinates of nodes visited
params{1,2} = T.Vr(end);    % Total distance travelled

%%% Display
if dispNodes
    plot(G.Nodes.x, G.Nodes.y, 'bo', ...
         'MarkerFaceColor', 'b', 'MarkerSize', 4);
end
if ~dispPRM 
    plot(s(1), s(2), 's', ...
        'MarkerEdgeColor', 'k', ...
        'MarkerSize', 6, ...
        'MarkerFaceColor', path_colour);
    plot(f(1), f(2), 's', ...
        'MarkerEdgeColor', 'k', ...
        'MarkerSize', 6, ...
        'MarkerFaceColor', path_colour);
    text(s(1), s(2), '\leftarrow START', 'HorizontalAlignment','left', 'Color', 'r');
    text(f(1), f(2), 'FINISH \rightarrow', 'HorizontalAlignment','right', 'Color', 'r');
end