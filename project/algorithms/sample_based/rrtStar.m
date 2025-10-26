function [exist, distances, parents] = rrtStar(G, map, node_initial, node_final)
% RRTSTAR Builds a tree using the RRT* (optimal Rapidly-exploring Random Tree) algorithm.
%
%   [EXIST, DISTANCES, PARENTS] = RRTSTAR(G, MAP, NODE_INITIAL, NODE_FINAL)
%   attempts to find an optimal path from NODE_INITIAL to NODE_FINAL
%   in the graph G by incrementally building a tree of collision-free paths
%   using random sampling, rewiring, and cost minimization.
%
%   Inputs:
%       G            - Graph object representing the RRT.
%       MAP          - Binary occupancy grid (1 = free, 0 = obstacle).
%       NODE_INITIAL - Node object with initial state [x, y].
%       NODE_FINAL   - Node object with final/goal state [x, y].
%
%   Outputs:
%       EXIST     - Boolean flag indicating whether a path to the goal was found.
%       DISTANCES - Vector of cumulative distances from the start to each node.
%       PARENTS   - Vector of parent node IDs for reconstructing the tree.
%
%   The algorithm improves upon RRT by locally rewiring nodes to ensure that
%   each node is connected through the lowest-cost path, producing asymptotically
%   optimal solutions.

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

        % Node creation
        ID_new = G.addNode(q_new);

        % Neighborhood selection
        IDs_near = G.getNearbyIDs(q_new, G.rewiring_radius)';
        % IDs_near = G.getNearbyIDs(q_new, G.rewiring_radius*(log(numel(G.nodes)) / numel(G.nodes))^(1/(ndims(map)+1)) )';
        
        % Parent choice & edge creation
        distances(ID_new, 1) = +inf;
        for ID_near = IDs_near
            q_near = G.getNodeByID(ID_near).state;
            dist = distances(ID_near) + Node.euclideanDistance(q_near, q_new);
            if (dist < distances(ID_new, 1) && G.isValidConnection(q_near, q_new, map))
                distances(ID_new, 1) = dist;
                parents(ID_new, 1) = ID_near;
            end
        end
        node_parent = G.getNodeByID(parents(ID_new));
        G.addEdge(node_parent.ID, ID_new, Node.euclideanDistance(node_parent.state, q_new));

        % Rewiring stage
        for ID_near = IDs_near
            q_near = G.getNodeByID(ID_near).state;
            dist = distances(ID_new) + Node.euclideanDistance(q_near, q_new);
            if (dist < distances(ID_near) && G.isValidConnection(q_new, q_near, map))
                edge = G.getEdgeByConnection(parents(ID_near), ID_near);
                G.edges(edge.ID).ID_A = ID_new;
                G.edges(edge.ID).ID_B = ID_near;
                G.edges(edge.ID).weight = Node.euclideanDistance(q_new, q_near);
                distances(ID_near) = dist;
                parents(ID_near) = ID_new;
            end
        end
        
    end

    iter = iter + 1;

    % if (G.isValidConnection(q_new, node_final.state, map, 2*Node.euclideanDistance(q_new, node_final.state)))
    %     break
    % end

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
