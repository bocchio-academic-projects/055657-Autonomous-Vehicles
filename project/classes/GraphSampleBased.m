classdef GraphSampleBased < Graph
    % GRAPHSAMPLEBASED Graph class for sampling-based motion planning.
    %
    %   Extends the base Graph class by including parameters and methods
    %   specific to sampling-based algorithms such as RRT and RRT*.
    %   It supports steering, connection validation, and neighbor search.

    properties
        max_iter         % Maximum number of iterations/samples
        step_length      % Maximum step length when extending the tree
        rewiring_radius  % Rewiring radius used in RRT* (optional)
    end

    methods
        function obj = GraphSampleBased(max_iter, step_length, rewiring_radius)
            % GRAPHSAMPLEBASED Constructor for sample-based planning graph.
            %
            %   OBJ = GRAPHSAMPLEBASED(MAX_ITER, STEP_LENGTH, REWIRING_RADIUS)
            %   initializes a graph with the given parameters.
            %
            %   If REWIRING_RADIUS is not provided, it defaults to 0.
            if nargin <= 2
                rewiring_radius = 0;
            end

            obj@Graph();
            obj.max_iter = max_iter;
            obj.step_length = step_length;
            obj.rewiring_radius = rewiring_radius;
        end

        function IDs = getNearbyIDs(obj, state, radius)
            % GETNEARBYIDs Returns nearby node IDs within a specified radius.
            %
            %   IDs = GETNEARBYIDs(OBJ, STATE, RADIUS) returns a list of node
            %   IDs whose states are within RADIUS of STATE. The result is sorted
            %   by ascending distance. If RADIUS is not provided, all nodes are considered.
            if nargin <= 2
                radius = +inf;
            end

            [IDs, states] = obj.getCachedData();
            dists = sqrt((states(:, 1) - state(1)).^2 + (states(:, 2) - state(2)).^2);

            valid_idx = find(dists ~= 0 & dists < radius);
            [~, sort_idx] = sort(dists(valid_idx));
            IDs = IDs(valid_idx(sort_idx));
        end

        function state_B = steerState(~, state_A, state_B, radius)
            % STEERSTATE Returns a new state in the direction from A to B, limited by a radius.
            %
            %   STATE_B = STEERSTATE(STATE_A, STATE_B, RADIUS) computes a new state
            %   that lies on the line from STATE_A to STATE_B, but no farther than RADIUS
            %   from STATE_A. This is used to control tree growth (e.g., in RRT).
            distance = Node.euclideanDistance(state_A, state_B);
            if (distance == 0)
                return 
            end
            state_B = state_A + radius * (state_B - state_A) / distance;
        end

        function is_valid = isValidConnection(obj, state_A, state_B, map, N)
            % ISVALIDCONNECTION Checks whether the straight-line path between two states is collision-free.
            %
            %   IS_VALID = ISVALIDCONNECTION(OBJ, STATE_A, STATE_B, MAP, N) checks
            %   whether the interpolated path from STATE_A to STATE_B collides with
            %   obstacles in the given MAP. The path is sampled at 3*N points.
            %
            %   MAP can be 2D or 3D. If N is not provided, the graph's step_length is used.
            if (nargin < 5)
                N = obj.step_length;
            end

            dim = ndims(map);
            q_interpolated = state_A(1:dim)' + linspace(0, 1, ceil(3*N)) .* (state_B(1:dim)' - state_A(1:dim)');
            q_probing = [floor(q_interpolated) ceil(q_interpolated)];

            switch dim
                case 2
                    for q = q_probing
                        if (any(q < 1) || any(q > size(map)') || map(q(1), q(2)) == 0)
                            is_valid = false;
                            return;
                        end
                    end
                case 3
                    for q = q_probing
                        if (any(q < 1) || any(q > size(map)') || map(q(1), q(2), q(3)) == 0)
                            is_valid = false;
                            return;
                        end
                    end
            end

            is_valid = true;
        end
    end
end
