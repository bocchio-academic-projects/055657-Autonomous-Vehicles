classdef Graph < handle
    % GRAPH Class for representing a graph structure with nodes and weighted edges.
    % Provides methods for adding and retrieving nodes, edges, and adjacency data.

    properties
        nodes Node
        edges Edge
        adjacents cell
    end

    methods
        function obj = Graph()
            obj.nodes = Node.empty;
            obj.edges = Edge.empty;
            obj.adjacents = {};
        end

        function ID = addNode(obj, state)
            ID = length(obj.nodes) + 1;
            obj.nodes(ID, 1) = Node(ID, state);
            obj.adjacents{ID, 1} = [];
        end

        function ID = addEdge(obj, ID_A, ID_B, weight)
            ID = length(obj.edges) + 1;
            obj.edges(ID, 1) = Edge(ID, ID_A, ID_B, weight);
            obj.adjacents{ID_A, 1}(end+1, 1) = ID_B;
        end

        function node = getNodeByID(obj, ID)
            node = obj.nodes(ID);
        end

        function edge = getEdgeByID(obj, ID)
            edge = obj.edges(ID);
        end


        function nodes = getNodesByState(obj, state)
            nodes = Node.empty;
            for node = obj.nodes'
                if Node.compareStates(node.state, state)
                    nodes(end+1) = node;
                end
            end
        end


        function edge = getEdgeByConnection(obj, ID_A, ID_B)
            for edge = obj.edges([obj.edges.ID_A] == ID_A)'
                if (edge.ID_B == ID_B)
                    return
                end
            end
        end


        function adjacents = getAdjacents(obj, node_ID)
            adjacents = obj.adjacents{node_ID};
        end
    end

end
