%% MAIN
clear;
close all;

%% Parameters
run parameters
% run linearModel

%% Set up field plotting
field = struct();
field.length = 20;  %x-coordinate
field.width = 20;   %y-coordinate
field.height = 20;  %z-coordinate
field.center = 0.5*[field.length, field.width, field.height]';
field.bound = [field.length, field.width, field.height]';

start = [0, 0, 0]';
goal = [field.length, field.width, field.height]';

figure
plot3(field.length, field.width, field.height)
xlim([0 field.length]);
ylim([0 field.width]);
zlim([0 field.height]);
xlabel('x');
ylabel('y');
zlabel('z');
grid on; grid minor;

%% Obstacle in 2D
% obstacleColor = [0 .5 .5];
% rectangle('Position',[0 7 9.5 1], 'FaceColor', obstacleColor)
% rectangle('Position',[10.5 7 9.5 1], 'FaceColor', obstacleColor)
% rectangle('Position',[5.25 10 9.5 1], 'FaceColor', obstacleColor)
% hold on;
% scatter(start(1), start(2), 'filled', 'MarkerEdgeColor', [0 0 0]);
% scatter(goal(1), goal(2), 'filled', 'MarkerEdgeColor', [0 0 0]);

%% Obstacle in 3D
hold on;
[shapes] = PlayingFieldV2(param.obs.amount,param.obs.size,[field.length field.width field.height],param.obs.verti);
scatter3(start(1), start(2), start(3), 'filled', 'MarkerEdgeColor', [0 0 0]);
text(start(1), start(2), start(3), '  Start node')

scatter3(goal(1), goal(2), goal(3), 'filled', 'MarkerEdgeColor', [0 0 0]);
text(goal(1), goal(2), goal(3), '  Target node')

N = 80;
nodes = zeros(param.dim, N+1);
edges = [];
path = [];
cost = inf(1, N+1); % Shortest distance of the current point to start

nodes(:, 1) = start;
cost(1) = 0; % Cost of the start is zero.
nodeDist = @(q1, q2) (sum((q2-q1).^2,1).^0.5);

gamma = 20; % can be calculated explicitely

i=2;
while i<=N+1
    % Sample random new point in the workspace
    if i==N+1
        q_new = goal;
    else
        q_new = field.bound.*rand(param.dim,1);
    end
    nodes(:, i) = q_new;
    % Check whether the random point collides with an obstacle
    coll_point = DroneInObstacle(q_new, shapes, param.drone.r);   %check if point is a valaible position for the drone
    % If the new point is not in collision, the best edges will be
    % determined, otherwise the point will be deleted.
    if ~coll_point
        % If there are points nearby, a connection will be made, if not the
        % point is deleted.
        [distance, nodes_near, ball_radius] = findNearNodes(nodes, q_new, param.dim, gamma, nodeDist);  % check if connection with closest node goes throug obstacle
        if ~isempty(nodes_near)
            % For all points within the ball radius, it is checked if the
            % edge between the new point and the other points collides with
            % an obstacle. And the cost for the new point is calculated and
            % the new point is connected to the point that results in the
            % lowest cost for the new point.
            cost_tmp = inf(length(nodes_near),1);
            coll_near_node = ones(length(nodes_near),1);
            for t=1:length(nodes_near)
                coll_near_node(t) = LineInObstacle(q_new,nodes(:,nodes_near(t)),shapes,param.drone.r);
                if ~coll_near_node(t)
                    cost_tmp(t) = cost(nodes_near(t)) + distance(t);
                end
            end
            if (length(coll_near_node) - nnz(coll_near_node)) > 0 % Als iemand iets beters hiervoor weet...
                [cost(i), idx_min_cost] = min(cost_tmp);
                new_edge = [nodes_near(idx_min_cost); i];
                edges = [edges new_edge]; % "connection via indices"
                nodes_near(idx_min_cost) = [];
                distance(idx_min_cost) = [];
            else
                nodes(:,i)=[0, 0, 0]';
                i=i-1;
            end
            
            % Rewire edges from previous points to the new point if that
            % results in a lower cost for the previous points (this is only
            % done for the point within the given ball radius)
            coll_near_node = ones(length(nodes_near),1);
            for t=1:length(nodes_near)
                coll_near_node(t)=LineInObstacle(nodes(:,idx_min_cost),nodes(:,nodes_near(t)),shapes,param.drone.r);
                if coll_near_node(t)==0
                    cost_tmp_1 = cost(i)+ distance(t);
                    if cost_tmp_1 < cost(nodes_near(t))
                        for s = 1:2
                            edges_del = find(edges(s,:)== nodes_near(t));
                        end
                        edges(:,edges_del) = []; 
                        new_edge = [nodes_near(t); i];
                        edges = [edges new_edge]; % "connection via indices"
                    end
                end
            end
        else
            if i==N+1
                disp('Cannot connect to goal')
                return
            else
                nodes(:,i)=[0, 0, 0]';
                i=i-1;
            end
        end
    else
        nodes(:,i)=[0, 0, 0]';
        i=i-1;
    end
    scatter3(nodes(1,i),nodes(2,i),nodes(3,i))
    text(nodes(1,i),nodes(2,i),nodes(3,i),num2str(i))
    i=i+1;
    disp(['i = ', num2str(i)])
end

% Plot the lines between the random generated points
for j=1:length(edges)
    line([nodes(1, edges(1, j)) nodes(1, edges(2, j))], ...
         [nodes(2, edges(1, j)) nodes(2, edges(2, j))], ...
         [nodes(3, edges(1, j)) nodes(3, edges(2, j))], ...
         'Color', 'red')
end

% Trace the shortest path back from goal to start
current_point = goal;
for s = 1:length(nodes)
    if isequal(nodes(:,s), current_point)
        node_idx = s;
        path = s;
    end
end

while cost(node_idx) > 0
    for s = 1:length(nodes)
        if isequal(nodes(:,s), current_point)
            edge_idx = s;
            new_point = inf(1, length(edges));
            for u = 1:length(edges)
                if edges(2,u) == edge_idx
                    new_point(u) = edges(1,u);
                end
                if edges(1,u) == edge_idx
                    new_point(u) = edges(2,u);
                end
            end
            for v=length(new_point):-1:1
                if ismember(new_point(v), path)
                    new_point(v)=[];
                end
            end
            node_idx = min(new_point);
            path = [path node_idx];
            current_point = nodes(:,node_idx);
        end
    end
end