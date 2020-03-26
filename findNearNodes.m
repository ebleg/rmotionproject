function [distance, nodes_near, ball_radius]=findNearNodes(nodes, q_new, dim, gamma, nodeDist)
% Returns the indices from the matrix 'nodes' that are within a given ball radius
% These indices are sorted from nearest nodes to farest away within the
% given ball radius

% TODO: Fix that the first random point is connected to the start if
% possible and that it is not possible that separate trees exists.

% dim = param.dim;
n = nnz(nodes)/dim + 1;
ball_radius = gamma * nthroot(log(n)/(n),dim);
Dist =[];
idx = [];
distance=[];
nodes_near =[];
for k = 1:n-1
    Dist(k,:) = nodeDist(q_new, nodes(:,k));
end
idx = find(Dist <= ball_radius);
distance = Dist(idx);
distance = sort(distance);
for j=1:length(distance)
    for i=1:length(Dist)
        if distance(j) == Dist(i)
            idx(j) = find(distance(j) == Dist);
        end
    end
end
nodes_near = idx;
end
