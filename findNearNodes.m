function [nodes_near]=findNearNodes(nodes, q_new, dim, gamma, nodeDist)
n = nnz(nodes)/dim;
ball_radius = gamma * nthroot(log(n)/(n),dim);
I = NaN(length(n));
for i = 1:length(n)
    Dist = nodeDist(nodes, q_new);
    if Dist <= ball_radius
        [~, I] = [Dist, indices];
    end
    [nodes_near] = I;
end

% [indices,dists] = findNeighborsInRadius(nodes,q_new,ball_radius);
% q_near = rangesearch([nodes(1) nodes(2)], [q_new(1) q_new(2)], ball_radius);
% q_near = str2double (q_near);
% [~, I] = min(nodeDist(q_new, q_near));
end
