function plotSequence(G, sequenceEdges)

X = G.nodes(sequenceEdges(1).ID_A).state(2);
Y = G.nodes(sequenceEdges(1).ID_A).state(1);

for node = G.nodes([sequenceEdges.ID_B])'

    X = [X node.state(2)];
    Y = [Y node.state(1)];

end

plot(X, Y, 'r-', 'LineWidth', 3, 'HandleVisibility', 'off');

initial_node = G.getNodeByID(sequenceEdges(1).ID_A);
final_node = G.getNodeByID(sequenceEdges(end).ID_B);

plot(initial_node.state(2), initial_node.state(1), 'g*', ...
    'MarkerSize', 10, ...
    'DisplayName', 'Initial node')
plot(final_node.state(2), final_node.state(1), 'r*', ...
    'MarkerSize', 10, ...
    'DisplayName', 'Final node')

axis ij

end