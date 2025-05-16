classdef GraphGridBased < Graph
    % GRAPHGRIDBASED Graph representation for grid-based motion planning.
    %
    %   This class extends the generic Graph class to specifically represent
    %   graphs based on grid connectivity. It maintains a list of adjacent
    %   node IDs for each node to support efficient access during search
    %   algorithms like Dijkstra or A*.

    properties
        adjacents cell   % Cell array: each element contains adjacent node IDs for each node
    end

    methods
        function obj = GraphGridBased()
            % GRAPHGRIDBASED Constructor for the grid-based graph.
            %   Initializes an empty graph and adjacency structure.
            obj@Graph();
            obj.adjacents = {};
        end

        function ID = addNode(obj, state)
            % ADDNODE Adds a node to the graph with a given state.
            %
            %   ID = ADDNODE(OBJ, STATE) adds a new node with position or
            %   configuration given by STATE. Returns the unique ID assigned
            %   to the new node.
            ID = length(obj.nodes) + 1;
            obj.nodes(ID, 1) = Node(ID, state);
            obj.adjacents{ID, 1} = [];
            obj.cached_IDs = [obj.cached_IDs; ID];
            obj.cached_states = [obj.cached_states; state];
        end

        function ID = addEdge(obj, ID_A, ID_B, weight)
            % ADDEDGE Adds a directed edge from node A to node B.
            %
            %   ID = ADDEDGE(OBJ, ID_A, ID_B, WEIGHT) adds a new edge from node
            %   ID_A to node ID_B with the specified WEIGHT. Returns the edge ID.
            ID = length(obj.edges) + 1;
            obj.edges(ID, 1) = Edge(ID, ID_A, ID_B, weight);
            obj.adjacents{ID_A, 1}(end+1, 1) = ID_B;
        end
        
        function adjacents = getAdjacents(obj, node_ID)
            % GETADJACENTS Returns all adjacent node IDs for a node.
            %
            %   ADJACENTS = GETADJACENTS(OBJ, NODE_ID) returns a vector of IDs
            %   of the nodes adjacent to the node with ID = NODE_ID.
            adjacents = obj.adjacents{node_ID};
        end
    end
end
