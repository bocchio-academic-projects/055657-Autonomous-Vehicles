classdef Graph < handle
    % GRAPH Class for representing a graph structure with nodes and weighted edges.
    %
    %   This class stores a list of Node and Edge objects, supporting
    %   operations such as adding nodes/edges, querying connections,
    %   reconstructing paths, and plotting the graph in 2D or 3D.
    %   It includes internal caching of IDs and states for performance.

    properties
        nodes Node   % Array of Node objects in the graph
        edges Edge   % Array of Edge objects representing directed connections
    end

    properties (Access = protected)
        cached_IDs        % Cached array of node IDs for fast access
        cached_states     % Cached matrix of node states
        is_cache_dirty = true  % Flag indicating if the cache needs to be rebuilt
    end

    methods
        function obj = Graph()
            % GRAPH Constructor initializes an empty graph.
            %
            %   Creates empty containers for nodes and edges.
            obj.nodes = Node.empty;
            obj.edges = Edge.empty;
        end

        function ID = addNode(obj, state)
            % ADDNODE Adds a new node to the graph with the given state.
            %
            %   ID = ADDNODE(STATE) adds a node with the provided state
            %   vector and returns its assigned unique ID.
            ID = length(obj.nodes) + 1;
            obj.nodes(ID, 1) = Node(ID, state);
            obj.cached_IDs = [obj.cached_IDs; ID];
            obj.cached_states = [obj.cached_states; state];
            % obj.is_cache_dirty = true;
        end

        function ID = addEdge(obj, ID_A, ID_B, weight)
            % ADDEDGE Adds a directed edge between two node IDs.
            %
            %   ID = ADDEDGE(ID_A, ID_B, WEIGHT) creates an edge from node
            %   ID_A to node ID_B with the specified weight and returns
            %   the edge's unique ID.
            ID = length(obj.edges) + 1;
            obj.edges(ID, 1) = Edge(ID, ID_A, ID_B, weight);
        end

        function node = getNodeByID(obj, ID)
            % GETNODEBYID Retrieves the Node object with the given ID.
            node = obj.nodes(ID);
        end

        function edge = getEdgeByID(obj, ID)
            % GETEDGEBYID Retrieves the Edge object with the given ID.
            edge = obj.edges(ID);
        end

        function nodes = getNodesByState(obj, state)
            % GETNODESBYSTATE Finds all nodes matching a given state vector.
            %
            %   NODES = GETNODESBYSTATE(STATE) returns an array of nodes
            %   whose state is exactly equal to the provided state.
            nodes = Node.empty;
            for node = obj.nodes'
                if Node.compareStates(node.state, state)
                    nodes(end+1) = node;
                end
            end
        end

        function edge = getEdgeByConnection(obj, ID_A, ID_B)
            % GETEDGEBYCONNECTION Returns edge from ID_A to ID_B if it exists.
            %
            %   EDGE = GETEDGEBYCONNECTION(ID_A, ID_B) searches the edge list
            %   for a connection from node ID_A to ID_B and returns it.
            for edge = obj.edges([obj.edges.ID_A] == ID_A)'
                if edge.ID_B == ID_B
                    return
                end
            end
        end

        function [IDs, states] = getCachedData(obj)
            % GETCACHEDDATA Returns cached node IDs and states.
            %
            %   If the cache is outdated, rebuilds it from the node list.
            if obj.is_cache_dirty
                obj.cached_IDs = [obj.nodes.ID]';
                obj.cached_states = reshape([obj.nodes.state], ...
                    numel(obj.nodes(1).state), [])';
                obj.is_cache_dirty = false;
            end
            IDs = obj.cached_IDs;
            states = obj.cached_states;
        end

        function [sequenceLength, sequenceNodes, sequenceEdges] = reconstructSequence(obj, ~, final_node, ~, parents)
            % RECONSTRUCTSEQUENCE Builds a path from a parent map.
            %
            %   [LENGTH, NODES, EDGES] = RECONSTRUCTSEQUENCE(~, FINAL_NODE, ~, PARENTS)
            %   returns the total length, node sequence, and edge sequence
            %   by backtracking from FINAL_NODE using the PARENTS array.

            sequenceNodes = [];
            sequenceEdges = [];

            current = final_node.ID;

            % Trace back node sequence
            while ~isnan(current)
                sequenceNodes = [obj.nodes(current), sequenceNodes];
                current = parents(current);
            end

            % Construct edge sequence
            for k = 1:length(sequenceNodes) - 1
                ID_A = sequenceNodes(k).ID;
                ID_B = sequenceNodes(k + 1).ID;
                ID_edge = getEdgeByConnection(obj, ID_A, ID_B).ID;
                sequenceEdges = [sequenceEdges, obj.edges(ID_edge)];
            end

            % Compute total weight of the sequence
            sequenceLength = sum([sequenceEdges.weight]);
        end

        function plotGraph(obj, dim)
            % PLOTGRAPH Visualizes the graph in 2D or 3D.
            %
            %   PLOTGRAPH(PLOT_NODES) plots graph edges.

            if (nargin < 2)
                dim = 2;
            end

            % Plot edges
            for edge = obj.edges'
                stateA = obj.nodes(edge.ID_A).state;
                stateB = obj.nodes(edge.ID_B).state;

                if dim == 2
                    % 2D plot
                    plot([stateA(2), stateB(2)], [stateA(1), stateB(1)], ...
                        'k-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
                elseif dim == 3
                    % 3D plot
                    plot3([stateA(1), stateB(1)], [stateA(2), stateB(2)], ...
                          [stateA(3), stateB(3)], 'k-', 'LineWidth', 1.5, ...
                          'HandleVisibility', 'off');
                else
                    error('Unsupported state dimension: %d', length(stateA));
                end
            end

        end

        function plotSequence(obj, sequenceEdges, dim)
            % PLOTSEQUENCE Plots a specific path given as a sequence of edges.
            %
            %   PLOTSEQUENCE(EDGES) plots the edges as a red path and marks
            %   the start and goal nodes in green and red stars, respectively.

            if (nargin < 3)
                dim = 2;
            end

            start_state = obj.nodes(sequenceEdges(1).ID_A).state;

            X = start_state(1);
            Y = start_state(2);
            if dim == 3
                Z = start_state(3);
            end

            % Build path coordinates
            for node = obj.nodes([sequenceEdges.ID_B])'
                X = [X, node.state(1)];
                Y = [Y, node.state(2)];
                if dim == 3
                    Z = [Z, node.state(3)];
                end
            end

            % Plot path
            if dim == 2
                plot(Y, X, 'r-', 'LineWidth', 3, 'HandleVisibility', 'off');
            elseif dim == 3
                plot3(X, Y, Z, 'r-', 'LineWidth', 3, 'HandleVisibility', 'off');
            else
                error('Unsupported state dimension: %d', dim);
            end

            % Highlight start and goal nodes
            initial_node = obj.getNodeByID(sequenceEdges(1).ID_A);
            final_node = obj.getNodeByID(sequenceEdges(end).ID_B);

            if dim == 2
                plot(initial_node.state(2), initial_node.state(1), 'g*', ...
                    'MarkerSize', 10, 'DisplayName', 'Initial node');
                plot(final_node.state(2), final_node.state(1), 'r*', ...
                    'MarkerSize', 10, 'DisplayName', 'Final node');
                axis ij;  % Flip Y-axis
            elseif dim == 3
                plot3(initial_node.state(1), initial_node.state(2), initial_node.state(3), ...
                      'g*', 'MarkerSize', 10, 'DisplayName', 'Initial node');
                plot3(final_node.state(1), final_node.state(2), final_node.state(3), ...
                      'r*', 'MarkerSize', 10, 'DisplayName', 'Final node');
            end
        end
    end
end
