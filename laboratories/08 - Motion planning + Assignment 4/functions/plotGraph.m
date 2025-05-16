function plotGraph(G, plot_nodes)


if nargin < 2
    plot_nodes = false;
end

% legend off

% Plot edges
for edge = G.edges'

    YX_A = G.nodes(edge.ID_A).state;
    YX_B = G.nodes(edge.ID_B).state;
    plot([YX_A(2), YX_B(2)], [YX_A(1), YX_B(1)], 'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');

end

% Plot nodes
if plot_nodes
    for node = G.nodes'

        YX = node.state;
        plot(YX(2), YX(1), 'bo', 'MarkerSize', 3, 'LineWidth', 2);
        text(YX(2)+0.2, YX(1)-0.2, num2str(node.ID), 'Color', 'b');

    end
end

end
