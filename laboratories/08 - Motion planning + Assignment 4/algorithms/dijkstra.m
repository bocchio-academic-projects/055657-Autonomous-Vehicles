function [exist, distances, parents] = dijkstra(G, node_initial, node_final)
% DIJKSTRA Finds the shortest path between two nodes in a graph
% using Dijkstra's algorithm.

N = length(G.nodes);
distances = inf(N, 1);
parents   = NaN(N, 1);
visited   = false(N, 1);

% Initialization
ID_current = node_initial.ID;
distances(ID_current) = 0;

while (ID_current ~= node_final.ID && ~all(visited))

    unvisited = find(~visited);
    [~, ID_current] = min(distances(unvisited));
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
        end

    end

end

exist = ~isinf(distances(node_final.ID));

end
