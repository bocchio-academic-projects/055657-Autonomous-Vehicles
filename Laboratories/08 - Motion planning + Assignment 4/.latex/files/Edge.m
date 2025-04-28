classdef Edge
    % EDGE Represents a weighted connection between two nodes in a graph.
    % Stores IDs of connected nodes and the edge weight.

    properties
        ID double
        ID_A double
        ID_B double
        weight double
    end

    methods
        function obj = Edge(ID, ID_A, ID_B, weight)
            obj.ID = ID;
            obj.ID_A = ID_A;
            obj.ID_B = ID_B;
            obj.weight = weight;
        end

        function isConnection = isConnectionTo(obj, ID_A, ID_B)
            isConnection = obj.ID_A == ID_A & obj.ID_B == ID_B;
        end
    end

end
