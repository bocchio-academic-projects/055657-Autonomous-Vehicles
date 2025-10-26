function [exist, distances, parents] = aStar(G, ~, node_initial, node_final)
% ASTAR Finds the shortest path between two nodes in a graph
% using the A* algorithm.
%
%   [EXIST, DISTANCES, PARENTS] = ASTAR(G, ~, NODE_INITIAL, NODE_FINAL)
%   computes the shortest path in graph G from NODE_INITIAL to NODE_FINAL
%   using the A* search algorithm, which combines Dijkstra's method with
%   a heuristic based on the Euclidean distance to the goal.
%
%   Inputs:
%       G            - Graph object with nodes and weighted edges.
%       ~            - Placeholder input (not used, included for compatibility).
%       NODE_INITIAL - Node object representing the start node.
%       NODE_FINAL   - Node object representing the goal node.
%
%   Outputs:
%       EXIST     - Boolean flag indicating whether a path to the goal exists.
%       DISTANCES - Vector of actual path distances from the start to each node.
%       PARENTS   - Vector of parent node IDs used to reconstruct the shortest path.
%
%   The A* algorithm improves on Dijkstra by prioritizing nodes with the lowest
%   estimated total cost to the goal (actual distance + heuristic). This results
%   in faster convergence when the heuristic is well-informed.

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