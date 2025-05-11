function G = map3DToGraph(binary_map, allow_diagonal_connection)
% MAP2DTOGRAPH Converts a 3D binary map into a bidirectional graph.
%
%   G = MAP2DTOGRAPH(BINARY_MAP, ALLOW_DIAGONAL_CONNECTION) takes a 3D binary matrix
%   where a value of 1 indicates free space and 0 indicates obstacles.
%
%   ALLOW_DIAGONAL_CONNECTION: if true, uses full 26-neighbor connectivity,
%   otherwise only 6-connectivity (axis-aligned neighbors).
%
%   Nodes are placed at [x, y, z] grid positions of each traversable cell.

if nargin < 2
    allow_diagonal_connection = false;
end

G = GridBased();

map_size = size(binary_map);
node_ID_matrix = zeros(map_size);

% Add nodes
for idx = find(binary_map == 1)'
    [x, y, z] = ind2sub(map_size, idx);
    node_ID_matrix(x, y, z) = G.addNode([x, y, z]);
end

% Add edges
for idx = find(binary_map == 1)'
    [x, y, z] = ind2sub(map_size, idx);
    node_ID = node_ID_matrix(x, y, z);
    node = G.getNodeByID(node_ID);

    % Get neighborhood
    [dX, dY, dZ] = ndgrid(-1:1, -1:1, -1:1);
    dX = dX(:); dY = dY(:); dZ = dZ(:);
    neighbors = [dX, dY, dZ];
    neighbors(all(neighbors == 0, 2), :) = [];  % remove (0,0,0)

    if ~allow_diagonal_connection
        % Keep only axis-aligned neighbors (Manhattan neighbors)
        neighbors = neighbors(sum(abs(neighbors), 2) == 1, :);
    end

    for k = 1:size(neighbors, 1)
        nx = x + neighbors(k,1);
        ny = y + neighbors(k,2);
        nz = z + neighbors(k,3);

        if nx >= 1 && nx <= map_size(1) && ...
           ny >= 1 && ny <= map_size(2) && ...
           nz >= 1 && nz <= map_size(3)

            neighbor_ID = node_ID_matrix(nx, ny, nz);
            if neighbor_ID ~= 0 && neighbor_ID ~= node_ID
                neighbor_node = G.getNodeByID(neighbor_ID);
                weight = Node.euclideanDistance(node.state, neighbor_node.state);
                G.addEdge(node_ID, neighbor_ID, weight);
            end
        end
    end
end

end
