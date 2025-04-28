function [exist, distances, parents] = astar(G, node_initial, node_final)
% ASTAR Finds the shortest path between two nodes in a graph
% using the A* algorithm.

N = length(G.nodes);
distances = inf(N, 1);
costs     = inf(N, 1);
parents   = NaN(N, 1);
visited   = false(N, 1);

% Initialization
ID_current = node_initial.ID;
distances(ID_current) = 0;
costs(ID_current) = Node.euclideanDistance(node_initial.state, node_final.state);

while (ID_current ~= node_final.ID && ~all(visited))

    unvisited = find(~visited);
    [~, ID_current] = min(costs(unvisited));
    ID_current = unvisited(ID_current);

    visited(ID_current) = true;

    % For each neighbor not yet visited
    IDs_neighbors = G.adjacents{ID_current};
    IDs_neighbors = IDs_neighbors(~visited(IDs_neighbors));
    for ID_neighbor = IDs_neighbors'

        distance = ...
            distances(ID_current) + ...
            G.getEdgeByConnection(ID_current, ID_neighbor).weight;

        if distance < distances(ID_neighbor)
            
            distances(ID_neighbor) = distance;
            parents(ID_neighbor) = ID_current;

            node_neighbor = G.getNodeByID(ID_neighbor);
            costs(ID_neighbor) = distance + Node.euclideanDistance(node_neighbor.state, node_final.state);
        
        end

    end

end

exist = ~isinf(distances(node_final.ID));

end