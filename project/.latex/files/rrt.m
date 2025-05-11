function [exist, distances, parents] = rrt(G, map, node_initial, node_final)
% RRT Builds a tree using the Rapidly-exploring Random Tree (RRT) algorithm.
%
%   [EXIST, DISTANCES, PARENTS] = RRT(G, MAP, NODE_INITIAL, NODE_FINAL)
%   attempts to grow a tree from NODE_INITIAL to NODE_FINAL using random
%   sampling and a step-limited steering function.
%
%   Inputs:
%       G            - Graph object representing the RRT.
%       MAP          - Binary occupancy grid (1 = free, 0 = obstacle).
%       NODE_INITIAL - Node object with initial state.
%       NODE_FINAL   - Node object with final/goal state.
%
%   Outputs:
%       EXIST     - Boolean flag indicating whether a path to the goal was found.
%       DISTANCES - Vector of cumulative distances from the start to each node.
%       PARENTS   - Vector of parent node IDs for reconstructing the tree.
%
%   The tree is expanded by selecting random points and connecting the closest
%   node in the tree toward them, checking each connection for validity.

q_new = node_initial.state;
distances = zeros(1, 1);
parents = NaN(1, 1);
iter = 0;

while (Node.euclideanDistance(q_new, node_final.state) > G.step_length && iter < G.max_iter)

    % Sampling stage
    q_rand = rand(1, ndims(map)) .* size(map);

    % Finding the nearest node
    IDs_near = G.getNearbyIDs(q_rand);
    ID_near = IDs_near(1);
    q_near = G.getNodeByID(ID_near).state;

    % Steering stage
    q_new = G.steerState(q_near, q_rand, G.step_length);

    % Collision checking and tree update
    if (G.isValidConnection(q_near, q_new, map))

        ID_new = G.addNode(q_new);
        dist = Node.euclideanDistance(q_near, q_new);
        G.addEdge(ID_near, ID_new, dist);

        parents(ID_new, 1) = ID_near;
        distances(ID_new, 1) = distances(ID_near) + dist;

    end

    iter = iter + 1;

end

exist = ~(iter == G.max_iter);

% Connection with final node
if (exist)
    ID_final = G.addNode(node_final.state);
    IDs_near = G.getNearbyIDs(node_final.state);
    ID_near = IDs_near(1);
    q_near = G.getNodeByID(ID_near).state;
    dist = Node.euclideanDistance(q_near, node_final.state);
    G.addEdge(ID_near, ID_final, dist);
    parents(ID_final, 1) = ID_near;
    distances(ID_final, 1) = distances(ID_near) + dist;
end

end

