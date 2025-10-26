function [exist, distances, parents] = depthFirstSearch(G, ~, node_initial, node_final)
% DEPTHFIRSTSEARCH Finds a path using Depth-First Search (DFS)
%
%   [exist, distances, parents] = depthFirstSearch(G, ~, node_initial, node_final)
%   explores the graph G from node_initial to node_final using DFS.
%
%   Inputs:
%       G             - Graph object with adjacency list (G.adjacents)
%       ~             - Placeholder input (not used, included for compatibility).
%       node_initial  - Initial node (struct with .ID and .state)
%       node_final    - Final node (struct with .ID and .state)
%
%   Outputs:
%       exist     - True if a path is found, false otherwise
%       distances - Vector of depths from initial node
%       parents   - Parent ID of each node for path reconstruction

N = length(G.nodes);
distances = inf(N, 1);
parents   = NaN(N, 1);
visited   = false(N, 1);

% Initialization
ID_current = node_initial.ID;
distances(ID_current) = 0;
stack = [ID_current];

while (ID_current ~= node_final.ID && ~isempty(stack))
    ID_current = stack(end);
    stack(end) = []; % Pop

    if visited(ID_current)
        continue;
    end
    visited(ID_current) = true;

    IDs_adjacent = G.adjacents{ID_current};
    IDs_adjacent = IDs_adjacent(~visited(IDs_adjacent));

    for ID_next = flip(IDs_adjacent) % Flip to mimic recursive DFS
        if ~visited(ID_next)
            distances(ID_next) = distances(ID_current) + 1;
            parents(ID_next) = ID_current;
            stack(end + 1) = ID_next; % Push
        end
    end
end

exist = ~isinf(distances(node_final.ID));

end
