function G = generatePRM(As, cs)
global Width Height nNodes conn dispPRM

fprintf('Generating PRM with %d nodes and %d connections\n', nNodes, conn)
w = waitbar(0, sprintf('Generating PRM with %d nodes and %d connections\n', nNodes, conn));

%%% create graph with randomly sampled nodes
G = graph;
x = rand(1, nNodes)*Width';
y = rand(1, nNodes)*Height';
G = addnode(G, table(x', y', 'VariableNames', {'x', 'y'}));

%%% clear nodes colliding with obstacles
ids = []; % list of nodes to remove
for i = 1:size(G.Nodes, 1)
    x = G.Nodes{i, {'x', 'y'}}';
    if CheckCollisionNode(x, As, cs)
        ids = [ids, i]; % add to removal list
    end
end
G = rmnode(G, ids);
fprintf('%d nodes removed due to collision\n', length(ids)); 

%%% link each node with candidate neighbours
for i = 1:size(G.Nodes, 1)
    w = waitbar(i/size(G.Nodes,1), w, sprintf('Processing Node %d of %d', i, size(G.Nodes,1)));
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
close(w); % Close waitbar
end

