% Path Planning in 2D and 3D Occupancy Maps
%
% This project implements motion planning algorithms on 2D and 3D occupancy
% maps using:
%   - Grid-based algorithms: Dijkstra, A*, (not tested: BFS, DFS)
%   - Sampling-based algorithms: RRT, RRT*, RRT Kinematic
% The system allows selection of different algorithms and maps, and 
% visualizes the resulting paths in both 2D and 3D scenarios.
%
% Author: Tommaso Bocchietti
% Date: 11/05/2025
%
% Instruction:
% Modify the parameters scenario_name and algorithm_name to choose both the 
% map or 3D scenario to plan for, and the algorithm to adopt. Basic statics 
% are reported in the console during the elaboration.
% Apply the hints reported in the project report in case of planning tests
% on 3D maps to speed-up improvement.
%
% Requires:
%   - /
%
% 055657 Autonomous Vehicles, A.Y. 2024/25, @Politecnico_di_Milano

clc
clear variables
close all

addpath( ...
    'algorithms\', ...
    'classes\', ...
    'functions\', ...
    'maps\3D\', ...
    'algorithms\sample_based\', ...
    'algorithms\grid_based\')

ALGO = struct( ...
    'DIJKSTRA', 'Dijkstra', ...
    'ASTAR', 'A*', ...
    'RRT', 'RRT', ...
    'RRTSTAR', 'RRT*', ...
    'RRTKINEMATIC', 'RRT Kinematic');

MAP = struct( ...
    'M01', '01', ...
    'M02', '02', ...
    'M03', '03', ...
    'M04', '04', ...
    'Mtest', 'test', ...
    'Mcircle', 'circle', ...
    'M3D_10', '3D_10', ...
    'M3D_20', '3D_20', ...
    'M3D_30', '3D_30');


%% Parameters

scenario_name = MAP.M01;
algorithm_name = ALGO.RRT;


%% Initial setup
[map, initial_state, final_state, img] = loadScenario(scenario_name);

switch (algorithm_name)

    case ALGO.DIJKSTRA
        if (ndims(map) == 2) %#ok<ISMAT>
            G = map2DToGraph(map, true);
        else
            G = map3DToGraph(map, false);
        end
        initial_node = G.getNodesByState(initial_state);
        final_node   = G.getNodesByState(final_state);
        graphSearchHandler = @dijkstra;

    case ALGO.ASTAR
        if (ndims(map) == 2) %#ok<ISMAT>
            G = map2DToGraph(map, false);
        else
            G = map3DToGraph(map, false);
        end
        initial_node = G.getNodesByState(initial_state);
        final_node   = G.getNodesByState(final_state);
        graphSearchHandler = @aStar;

    case ALGO.RRT
        G = GraphSampleBased(5000, 1.0);
        initial_node = G.getNodeByID(G.addNode(initial_state));
        final_node   = Node(-1, final_state);
        graphSearchHandler = @rrt;

    case ALGO.RRTSTAR
        G = GraphSampleBased(5000, 1.0, 1.5);
        initial_node = G.getNodeByID(G.addNode(initial_state));
        final_node   = Node(-1, final_state);
        graphSearchHandler = @rrtStar;

    case ALGO.RRTKINEMATIC
        G = GraphSampleBased(10000, 1.0);
        initial_node = G.getNodeByID(G.addNode([initial_state, 0]));
        final_node   = Node(-1, [final_state, 0]);
        graphSearchHandler = @(G, map, initial_node, final_node) rrtKinematic(G, map, initial_node, final_node, @modelBicycle);

    otherwise
        error('Unknown searching algorithm: %s', algorithm_name);

end

printHeader([scenario_name '_' algorithm_name])


%% Path search

tic
[exist, distances, parents] = graphSearchHandler(G, map, initial_node, final_node);
assert(exist, 'No path found. Exiting.')
disp(['Graph search: ' sprintf('\t\t\t\t%.0f', 1e3*toc) 'ms'])
disp(['Graph size: ' sprintf('\t\t\t\t%d', numel(G.nodes)) '#'])


%% Sequence reconstruction

if (final_node.ID == -1)
    final_node = G.nodes(end);
end

tic
[sequenceLength, sequenceNodes, sequenceEdges] = G.reconstructSequence(initial_node, final_node, distances, parents);
disp(['Sequence reconstruction: ' sprintf('\t%.f', 1e3*toc) 'ms'])
disp(['Sequence length: ' sprintf('\t\t\t%.0f', sequenceLength), 'm'])


%% Plots

reset(0)
set(0, 'DefaultFigureNumberTitle', 'off');
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'defaultaxesfontsize', 12);

figure('Name', 'Graph')

switch(ndims(map))
    case 2

        tiles = tiledlayout(1, 1);

        graph_tile = nexttile(tiles);
        hold on
        grid on

        image(map * 255);
        G.plotGraph(2)
        G.plotSequence(sequenceEdges, 2)

        axis ij
        axis equal
        axis padded
        xlim([0 size(map, 1)+1])
        ylim([0 size(map, 2)+1])

        title([algorithm_name ' algorithm'])

    case 3

        figure('Name', '3D Scenario')
        tiles = tiledlayout(3, 6);

        viewer_tile = nexttile(tiles, [3, 3]);
        hold on
        grid on

        G.plotSequence(sequenceEdges, 3)

        [x, y, z] = meshgrid(1:size(map, 2), 1:size(map, 1), 1:size(map, 3));
        p = patch(isosurface(y, x, z, map, 0.5));
        p.FaceColor = 'black';
        p.EdgeColor = 'none';
        camlight;
        lighting gouraud;

        daspect([1 1 1]);
        view(3)

        xlim([0 size(map, 1)+1])
        ylim([0 size(map, 2)+1])
        zlim([0 size(map, 3)+1])

        title('3D Scenario with obstacles');
        xlabel('X')
        ylabel('Y')
        zlabel('Z')


        for z_idx = round(linspace(1, size(map, 3)-1, 9))

            nexttile(tiles)
            hold on

            imagesc(permute(map(:, :, z_idx), [2, 1]));

            axis equal tight;
            colormap(gray);

            title(['Z = ', num2str(z_idx)]);

            xlabel('X');
            ylabel('Y');

            nodes_in_slice = sequenceNodes(arrayfun(@(n) ceil(n.state(3)) == z_idx | floor(n.state(3)) == z_idx, sequenceNodes));
            for node = nodes_in_slice
                x = node.state(1);
                y = node.state(2);
                plot(x, y, 'ro', 'MarkerSize', 5, 'LineWidth', 1.5);
            end

        end

end



%% Functions

function printHeader(algorithm_name, total_width)
% PRINTHEADER Displays a formatted header line with centered algorithm name.
%
%   PRINTHEADER(ALGORITHM_NAME) prints a header line containing the given
%   ALGORITHM_NAME centered between dashes. The total line width defaults to 34.
%
%   PRINTHEADER(ALGORITHM_NAME, TOTAL_WIDTH) allows specification of the total
%   header width. The algorithm name is centered with dashes padding both sides.
%
%   Inputs:
%       ALGORITHM_NAME - String representing the name of the algorithm.
%       TOTAL_WIDTH    - (Optional) Total width of the header line (default: 34).
%
%   Example:
%       printHeader('A* Search');
%       % Output: ----------A* Search-----------

if nargin < 2
    total_width = 34;
end

name_len = length(algorithm_name);
dash_len = floor((total_width - name_len) / 2);
disp([ ...
    repmat('-', 1, dash_len), ...
    algorithm_name, ...
    repmat('-', 1, total_width - dash_len - name_len) ...
    ]);

end


function [map, initial_state, final_state] = reduce3DMap(map, initial_state, final_state) %#ok<DEFNU>
% REDUCE3DSCENARIO Reduces the resolution of a 3D occupancy map.
%
%   [MAP, INITIAL_STATE, FINAL_STATE] = REDUCE3DSCENARIO(MAP, INITIAL_STATE, FINAL_STATE)
%   reduces the resolution of the provided 3D binary occupancy map by a factor of 2 
%   in each dimension (assuming the map size is divisible by 2). The initial and final 
%   states are adjusted accordingly to the new resolution.
%
%   The function reshapes the map into smaller blocks, applying a minimum filter 
%   across each 2x2x2 block to reduce the map resolution. It also verifies that 
%   the initial and final states are within free space after the reduction.
%
%   Inputs:
%       MAP           - A 3D binary occupancy grid (1 = free, 0 = obstacle).
%       INITIAL_STATE - Starting [row, col, height] position (1-indexed).
%       FINAL_STATE   - Goal [row, col, height] position (1-indexed).
%
%   Outputs:
%       MAP           - A reduced resolution 3D binary occupancy grid.
%       INITIAL_STATE - Adjusted starting position in the reduced map.
%       FINAL_STATE   - Adjusted goal position in the reduced map.

% Define block size for reduction
blockSize = 2;

% Get the size of the map
sz = size(map);

% Ensure the map size is divisible by the block size
assert(all(mod(sz, blockSize) == 0), 'Map size must be divisible by block size');

% Reshape the map into smaller blocks
map = reshape(map, blockSize, sz(1)/blockSize, blockSize, sz(2)/blockSize, blockSize, sz(3)/blockSize);
map = permute(map, [2 4 6 1 3 5]);

% Apply a minimum filter across each block to reduce the resolution
map = squeeze(min(min(min(map, [], 4), [], 5), [], 6));

% Adjust the initial and final states according to the new map resolution
initial_state = round(initial_state / blockSize) + 1;
final_state = round(final_state / blockSize);

% Ensure the initial and final states are within free space
assert(map(initial_state(1), initial_state(2), initial_state(3)) == 1, 'Initial state not in free space');
assert(map(final_state(1), final_state(2), final_state(3)) == 1, 'Final state not in free space');

end
