function [exist, distances, parents] = breathFirstSearch(G, ~, node_initial, node_final)
% BREATHFIRSTSEARCH Finds a path using Breadth-First Search (BFS)
%
%   [exist, distances, parents] = breathFirstSearch(G, map, node_initial, node_final)
%   explores the graph G from node_initial to node_final using BFS.
%   The function assumes G.adjacents is a cell array of adjacency lists.
%
%   Inputs:
%       G             - Graph object with adjacency list (G.adjacents)
%       ~             - Placeholder input (not used, included for compatibility).
%       node_initial  - Initial node (struct with .ID and .state)
%       node_final    - Final node (struct with .ID and .state)
%
%   Outputs:
%       exist     - True if a path is found, false otherwise
%       distances - Vector of distances from initial node
%       parents   - Parent ID of each node for path reconstruction

N = length(G.nodes);
distances = inf(N, 1);
parents   = NaN(N, 1);
visited   = false(N, 1);

% Initialization
ID_current = node_initial.ID;
distances(ID_current) = 0;
visited(ID_current) = true;
queue = [ID_current];

while (ID_current ~= node_final.ID && ~isempty(queue))
    ID_current = queue(1);
    queue(1) = []; % Dequeue

    IDs_adjacent = G.adjacents{ID_current};
    IDs_adjacent = IDs_adjacent(~visited(IDs_adjacent));

    for ID_next = IDs_adjacent
        visited(ID_next) = true;
        distances(ID_next) = distances(ID_current) + 1;
        parents(ID_next) = ID_current;
        queue(end+1) = ID_next; % Enqueue
    end
end

exist = ~isinf(distances(node_final.ID));

end
