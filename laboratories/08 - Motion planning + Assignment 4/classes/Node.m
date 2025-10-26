classdef Node
    % NODE Represents a graph node with a unique ID and a state vector.
    % Includes utility methods for comparison and distance calculation.

    properties

        ID double
        state double

    end



    methods

        function obj = Node(ID, state)

            obj.ID = ID;
            obj.state = state;

        end

        
        function isEqual = eq(node_A, node_B)

            isEqual = Node.compareStates(node_A.state, node_B.state);

        end

    end



    methods (Static)

        function dist = euclideanDistance(state_A, state_B)

            dist = norm(state_A - state_B);

        end


        function equal = compareStates(state_A, state_B)

            equal = isequal(state_A, state_B);

        end
        
    end

end
