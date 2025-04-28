% clc
% clear variables
% close all

%% Parameters

image_name = '02';
algorithm_name = 'dijkstra';
% algorithm_name = 'astar';
allow_diagonal_connection = true;


%% Initial setup
[map, initial_state, final_state, img] = loadImg(image_name);

switch lower(algorithm_name)
    case 'dijkstra'
        graphSearchHandler = @dijkstra;
    case 'astar'
        graphSearchHandler = @astar;
    otherwise
        error('Unknown algorithm: %s', algorithm_name);
end


%% Graph generation

tic
G = imgToGraph(map, allow_diagonal_connection);
disp(['Graph generation: ' sprintf('\t%.3f', toc) 's'])


%% Path search

initial_node = G.getNodesByState(initial_state);
final_node  = G.getNodesByState(final_state);
assert(~isempty(initial_node) & ~isempty(final_node), 'Undefined nodes.')

tic
[sequenceFound, distances, parents] = graphSearchHandler(G, initial_node, final_node);
assert(sequenceFound, 'No connection found between nodes.')
disp(['Graph search: ' sprintf('\t\t%.3f', toc) 's'])


%% Sequence reconstruction

tic
[sequenceLength, sequenceNodes, sequenceEdges] = reconstructSequence(G, initial_node, final_node, distances, parents);
disp(['Sequence reconstruction: ' sprintf('\t%.3f', toc) 's'])


%% Results

fprintf('\n')
fprintf([ ...
    'Results:\n' ...
    '\tNodes analyzed: %d\n' ...
    '\tPath length: %.0f\n'], ...
    sum(isfinite(distances)), sequenceLength);



% %% Plots
% reset(0)
% set(0, 'DefaultFigureNumberTitle', 'off');
% set(0, 'DefaultFigureWindowStyle', 'docked');
% set(0, 'DefaultLineLineWidth', 1.2);
% set(0, 'defaultaxesfontsize', 12);
% 
% % Waypoints visualization
% figure('Name', 'Graph')
% tiles = tiledlayout(1, 2);
% 
% map_tile = nexttile(tiles, 1);
% 
% if exist('img', 'var')
%     imshow(img)
% end
% 
% title('Full size map')
% 
% graph_tile = nexttile(tiles, 2);
% hold on
% grid on
% 
% plotGraph(G)
% plotSequence(G, sequenceEdges)
% 
% % axis equal
% axis padded
% 
% title('Graph Visualization')
% 
% 
% %% Export
% 
% connection_type = 'orthogonal';
% if allow_diagonal_connection
%     connection_type = 'diagonal';
% end
% exportgraphics(graph_tile, ['.latex/img/MATLAB/' image_name '_' algorithm_name '_' connection_type '.pdf']);
