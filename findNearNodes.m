function [nodes_near]=findNearNodes(nodes, q_new, dim, gamma)
n = nnz(nodes);
ball_radius = gamma * nthroot(log(n)/(n),dim);
[indices,dists] = findNeighborsInRadius(ptCloud,point,radius);
% q_near = rangesearch([nodes(1) nodes(2)], [q_new(1) q_new(2)], ball_radius);
% q_near = str2double (q_near);
% [~, I] = min(nodeDist(q_new, q_near));
end
