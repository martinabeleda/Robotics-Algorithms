function T = Dijkstra(graph, src)
%Finds the optimal path from a source node to all other nodes in the graph 
%using Dijkstra's algorithm.
% Arguments:
%   graph = NxN Directed adjacency matrix
%   source_node = Desired source node which is an index of graph
%
% Returns:
%   T = Table containing:
%       V_r -  cost to reach each node from source node
%       pi  - previous node

fprintf('Computing shortest path\n');

n = size(graph,1);
T = table(inf(n,1), zeros(n,1), inf(n,1), false(n,1), ...
          'VariableNames', {'Vr', 'pi', 'temp', 'visited'});
T.Vr(src) = 0; T.temp(src) = 0;
visits = 0;

% Continue until all nodes visited - when Vr contains no Inf
while visits <= n %~all(T.visited,1)
    [~, x] = min(T.temp); % Get the index 'x' of the minimum value 
    T.visited(x) = true;
    T.temp(x) = Inf; % Replace cost with Inf so does not appear as min
    for y = find(graph(x,:) > 0) % find the nodes adjacent to x
        if T.Vr(x) + graph(x,y) < T.Vr(y)
            T.Vr(y) = T.Vr(x) + graph(x,y);
            T.temp(y) = T.Vr(x) + graph(x,y);
            T.pi(y) = x;
        end
    end
    visits = visits + 1;
end
T = T(:,{'Vr', 'pi'}); % Remove temp column
end

