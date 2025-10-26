function [exist, distances, parents] = rrtKinematic(G, map, node_initial, node_final, vehicle_model)
% RRTKINEMATIC Builds a tree using RRT with vehicle kinematic constraints.
%
%   [EXIST, DISTANCES, PARENTS] = RRTKINEMATIC(G, MAP, NODE_INITIAL, NODE_FINAL, VEHICLE_MODEL)
%   attempts to find a path from NODE_INITIAL to NODE_FINAL using a kinematically
%   feasible motion model for non-holonomic vehicles (e.g., cars).
%
%   Inputs:
%       G             - Graph object to store the RRT tree.
%       MAP           - Binary occupancy map (1 = free, 0 = obstacle).
%       NODE_INITIAL  - Starting node with state [x, y, θ].
%       NODE_FINAL    - Goal node with target state [x, y, θ].
%       VEHICLE_MODEL - Function handle simulating vehicle motion over time.
%
%   Outputs:
%       EXIST     - Boolean indicating whether a valid path was found.
%       DISTANCES - Cumulative cost from the start node to each tree node.
%       PARENTS   - Parent indices used to reconstruct the tree structure.
%
%   This version of RRT uses motion primitives derived from a forward vehicle model
%   to ensure that the generated paths respect the system’s kinematic constraints.

v_options = linspace(-1, 1, 2);
v_options = 1;
delta_options = linspace(-pi/6, pi/6, 3);

q_new = node_initial.state;
distances = zeros(1, 1);
parents = NaN(1, 1);
iter = 0;

while (Node.euclideanDistance(q_new(1:2), node_final.state(1:2)) > G.step_length && iter < G.max_iter)

    % Sampling stage
    q_rand = [max(rand(1, 2) .* size(map), 1), rand() * 2 * pi];

    % Finding the nearest node
    IDs_near = G.getNearbyIDs(q_rand);
    ID_near = IDs_near(1);
    q_near = G.getNodeByID(ID_near).state;

    % Kinematics constraints
    best_state = [];
    min_dist = Inf;
    for v = v_options
        for delta = delta_options
    
            % Forward integration of vehicle motion
            q_new = vehicle_model(q_near, [v, delta], G.step_length / abs(v));
            
            % Collision checking and choice of nearest state to sampled one
            if (G.isValidConnection(q_near, q_new, map))
                dist = Node.euclideanDistance(q_new(1:2), q_rand(1:2));
                if (dist < min_dist)
                    best_state = q_new;
                    min_dist = dist;
                end
            end
    
        end
    end

    if isempty(best_state)
        continue;
    end

    % Tree update
    q_new = best_state;
    ID_new = G.addNode(q_new);
    dist = Node.euclideanDistance(q_near(1:2), q_new(1:2));
    G.addEdge(ID_near, ID_new, dist);

    parents(ID_new, 1) = ID_near;
    distances(ID_new, 1) = distances(ID_near) + dist;

    iter = iter + 1;

end

exist = ~(iter == G.max_iter);

% Connection with final node
if (exist)
    ID_final = G.addNode(node_final.state);
    IDs_near = G.getNearbyIDs(node_final.state);
    ID_near = IDs_near(1);
    q_near = G.getNodeByID(ID_near).state;
    dist = Node.euclideanDistance(q_near(1:2), node_final.state(1:2));
    G.addEdge(ID_near, ID_final, dist);
    parents(ID_final, 1) = ID_near;
    distances(ID_final, 1) = distances(ID_near) + dist;
end

end