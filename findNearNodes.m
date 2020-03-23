function [nodes_near]=findNearNodes(nodes, q_new, dim, gamma, nodeDist)
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
