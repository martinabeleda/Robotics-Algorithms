function WP = OptimalPath(T, G, finish, draw, path_colour)
%Uses a greedy algorithm to trace the optimal path derived by Dijkstra or
%A*. Assuming that Vr is the cost to reach a certain node and pi points to
%the previous node, this traveses backwards from the finish to the start
%node.
%
% Arguments:
%   T = Table containing:
%       V_r -  cost to reach each node from source node
%       pi  - previous node
%   finish = destination node index in G
%
% Returns:
%   WP = waypoints in (x,y) from start to finish

global dispPRM
nodes = finish; % traverse starting from the finish node
WP = G.Nodes{finish, {'x','y'}}'; % corresponding (x,y) co-ordinate
dist = [];
% traverse the graph until we reach the start node
while true   
    next = T.pi(nodes(end));
    if next == 0, break, end % we have reached the start node
    nodes = [nodes next];
    WP = [WP G.Nodes{next, {'x','y'}}'];
    if draw % plot this edge
        x = [G.Nodes{nodes(end),1} G.Nodes{nodes(end-1),1}];
        y = [G.Nodes{nodes(end),2} G.Nodes{nodes(end-1),2}];
        if dispPRM
            plot(x, y, 'b-', 'LineWidth', 2.5);
            plot(x, y, '-', 'LineWidth', 1.5, 'Color', path_colour);
        else
            plot(x, y, '--', 'LineWidth', 1, 'Color', path_colour);
        end
    end
end

WP = fliplr(WP);
end

