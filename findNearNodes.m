function [nodes_near]=findNearNodes(nodes, q_new, dim, gamma, nodeDist)
% Returns the indices from the matrix 'nodes' that are within a given ball radius
% These indices are sorted from nearest nodes to farest away within the
% given ball radius

% dim = param.dim;
n = nnz(nodes)/dim;
ball_radius = gamma * nthroot(log(n)/(n),dim);
% ball_radius=1;
Dist =[];
idx = [];
nodes_near =[];
for k = 1:n-1
    Dist(k) = nodeDist(q_new, nodes(:,k));
end
for j = 1:length(Dist)
    if Dist(j) <= ball_radius
        [~, idx] = sort(Dist); 
    end
end
nodes_near = idx;
end
