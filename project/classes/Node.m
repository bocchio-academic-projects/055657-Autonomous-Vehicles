classdef Node
    % NODE Represents a graph node with a unique ID and a state vector.
    %
    %   Each node encapsulates an identifier and a state, typically used
    %   for spatial coordinates (e.g., [x, y] or [x, y, z]) in planning or
    %   search algorithms. Includes utility methods for comparison and
    %   distance measurement.

    properties
        ID double      % Unique identifier for the node
        state double   % State vector (e.g., [x, y] position or similar)
    end

    methods
        function obj = Node(ID, state)
            % NODE Constructor for the Node class.
            %
            %   OBJ = NODE(ID, STATE) creates a node object with the
            %   specified unique identifier and associated state vector.
            obj.ID = ID;
            obj.state = state;
        end

        function isEqual = eq(node_A, node_B)
            % EQ Overloaded '==' operator to compare nodes by state.
            %
            %   ISEQUAL = EQ(NODE_A, NODE_B) returns true if the states of
            %   both nodes are exactly equal.
            isEqual = Node.compareStates(node_A.state, node_B.state);
        end
    end

    methods (Static)
        function dist = euclideanDistance(state_A, state_B)
            % EUCLIDEANDISTANCE Computes the Euclidean distance between two states.
            %
            %   DIST = EUCLIDEANDISTANCE(STATE_A, STATE_B) returns the
            %   Euclidean norm of the difference between two state vectors.
            dist = norm(state_A - state_B);
        end

        function equal = compareStates(state_A, state_B)
            % COMPARESTATES Checks for equality between two state vectors.
            %
            %   EQUAL = COMPARESTATES(STATE_A, STATE_B) returns true if the
            %   state vectors are exactly equal (element-wise).
            equal = isequal(state_A, state_B);
        end
    end
end
