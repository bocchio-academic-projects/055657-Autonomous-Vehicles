function [sequenceLength, sequenceNodes, sequenceEdges] = reconstructSequence(G, ~, final_node, distances, parents)

sequenceLength = 0;
sequenceNodes = [];
sequenceEdges = [];

current = final_node.ID;

% Node sequence reconstruction
while ~isnan(current)

    sequenceNodes = [G.nodes(current), sequenceNodes];
    current = parents(current);

end

% Edge sequence reconstruction
for k = 1:length(sequenceNodes)-1

    A = sequenceNodes(k).ID;
    B = sequenceNodes(k+1).ID;

    edge_idx = find([G.edges.ID_A] == A & [G.edges.ID_B] == B);
    sequenceEdges = [sequenceEdges, G.edges(edge_idx)];

end

% Path length computation
sequenceLength = sum([sequenceEdges.weight]);

end

