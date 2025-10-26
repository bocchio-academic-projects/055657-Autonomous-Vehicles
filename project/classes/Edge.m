classdef Edge
    % EDGE Represents a directed, weighted edge in a graph.
    %
    %   Each edge stores its unique ID, the IDs of the two nodes it connects
    %   (from ID_A to ID_B), and the weight associated with the edge, which
    %   typically represents a distance, cost, or time.

    properties
        ID double       % Unique identifier for the edge
        ID_A double     % ID of the source/start node
        ID_B double     % ID of the target/end node
        weight double   % Weight of the edge (e.g., cost or distance)
    end

    methods
        function obj = Edge(ID, ID_A, ID_B, weight)
            % EDGE Constructor for an Edge object.
            %
            %   OBJ = EDGE(ID, ID_A, ID_B, WEIGHT) creates a new edge with
            %   the specified unique identifier, source and target node IDs,
            %   and associated weight.
            obj.ID = ID;
            obj.ID_A = ID_A;
            obj.ID_B = ID_B;
            obj.weight = weight;
        end

        function isConnection = isConnectionTo(obj, ID_A, ID_B)
            % ISCONNECTIONTO Checks whether this edge connects two specific nodes.
            %
            %   IS_CONNECTION = ISCONNECTIONTO(OBJ, ID_A, ID_B) returns true
            %   if this edge starts at node ID_A and ends at node ID_B.
            isConnection = obj.ID_A == ID_A && obj.ID_B == ID_B;
        end
    end
end
