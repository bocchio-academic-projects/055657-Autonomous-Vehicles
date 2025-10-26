function [exist, distances, parents] = dijkstra(G, ~, node_initial, node_final)
% DIJKSTRA Finds the shortest path between two nodes in a graph
% using Dijkstra's algorithm.
%
%   [EXIST, DISTANCES, PARENTS] = DIJKSTRA(G, ~, NODE_INITIAL, NODE_FINAL)
%   computes the shortest path in graph G from NODE_INITIAL to NODE_FINAL
%   using Dijkstra's algorithm.
%
%   Inputs:
%       G            - Graph object with nodes and weighted edges.
%       ~            - Placeholder input (not used, included for compatibility).
%       NODE_INITIAL - Node object representing the start node.
%       NODE_FINAL   - Node object representing the goal node.
%
%   Outputs:
%       EXIST     - Boolean flag indicating whether a path to the goal exists.
%       DISTANCES - Vector of shortest distances from the start node to each node.
%       PARENTS   - Vector containing the parent node ID for each node in the path.
%
%   The algorithm initializes distances from the start node to all others as
%   infinite, then iteratively visits the closest unvisited node and updates
%   distances to its neighbors. The shortest path is reconstructed using the
%   PARENTS vector.


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
