function G = map2DToGraph(binary_map, allow_diagonal_connection)
% MAP2DTOGRAPH Converts a binary map into a bidirectional graph.
%
%   G = MAP2DTOGRAPH(BINARY_MAP, ALLOW_DIAGONAL_CONNECTION) takes a 2D binary matrix
%   BINARY_MAP where a value of 1 indicates a free (traversable) cell and 0 
%   indicates an obstacle. It creates a Graph object G in which each traversable 
%   cell is a node, and edges are added between adjacent nodes.
%
%   ALLOW_DIAGONAL_CONNECTION is an optional boolean argument that enables 
%   diagonal connectivity. If true, 8-connectivity is used; otherwise, only 
%   4-connectivity is applied. Default is false.
%
%   Nodes are placed at the (row, col) coordinates of each traversable cell.
%   Edge weights are based on the Euclidean distance between node positions.
%
%   Example:
%       map = [1 1 0; 0 1 1; 1 0 1];
%       G = imgToGraph(map, true);

if nargin < 2
    allow_diagonal_connection = false;
end

G = GraphGridBased();

% Nodes creation
img_size = size(binary_map);
node_ID_matrix = zeros(img_size);

for idx = find(binary_map == 1)'

    [row, col] = ind2sub(img_size, idx);
    node_ID_matrix(row, col) = G.addNode([row, col]);

end


% Edges creation
for idx = find(binary_map == 1)'

    [row, col] = ind2sub(img_size, idx);
    node_ID = node_ID_matrix(row, col);
    node = G.getNodeByID(node_ID);

    [row_indices, col_indices] = meshgrid( ...
        [-1 0 +1] + row, ...
        [-1 0 +1] + col);

    % Valid bounds check
    boundary_mask = ...
        row_indices >= 1 & row_indices <= img_size(1) & ...
        col_indices >= 1 & col_indices <= img_size(2);

    % Diagonal mask
    diagonal_mask = ...
        abs(row_indices - row) ~= 1 | ...
        abs(col_indices - col) ~= 1;

    mask = ...
        boundary_mask & ...
        ( ...
            allow_diagonal_connection * ones(size(boundary_mask)) + ...
            ~allow_diagonal_connection * diagonal_mask ...
        );

    row_indices = row_indices(mask);
    col_indices = col_indices(mask);

    neighbor_IDs = node_ID_matrix(sub2ind(size(node_ID_matrix), row_indices(:), col_indices(:)));
    neighbor_IDs = neighbor_IDs(neighbor_IDs ~= 0 & neighbor_IDs ~= node_ID);

    for neighbor_ID = neighbor_IDs'
    
        neighbor_node = G.getNodeByID(neighbor_ID);
        weight = Node.euclideanDistance(node.state, neighbor_node.state);
        G.addEdge(node_ID, neighbor_ID, weight);
    
    end

end

end
