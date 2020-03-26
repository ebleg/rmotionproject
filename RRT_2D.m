%% MAIN
clear;
close all;

%% Parameters
run parameters
param.dim = 2;
% run linearModel

%% Set up field plotting
field = struct();
field.length = 20;  %x-coordinate
field.width = 20;   %y-coordinate
field.center = 0.5*[field.length, field.width]';
field.bound = [field.length, field.width]';

start = [0, 0]';
goal = [field.length, field.width]';

figure
plot(field.length, field.width)
xlim([0 field.length]);
ylim([0 field.width]);
xlabel('x');
ylabel('y');
grid on; grid minor;
axis equal

%% Obstacle in 2D
obstacleColor = [0 .5 .5];
% rectangle('Position',[0 7 9.5 1], 'FaceColor', obstacleColor)
% rectangle('Position',[10.5 7 9.5 1], 'FaceColor', obstacleColor)
% rectangle('Position',[5.25 10 9.5 1], 'FaceColor', obstacleColor)
hold on;
scatter(start(1), start(2), 'filled', 'MarkerEdgeColor', [0 0 0]);
scatter(goal(1), goal(2), 'filled', 'MarkerEdgeColor', [0 0 0]);

%%
N = 5e1;
nodes = zeros(param.dim, N+1);
edges = [];
path = [];
cost = inf(1, N+1); % Shortest distance of the current point to start

nodes(:, 1) = start;
cost(1) = 0; % Cost of the start is zero.
nodeDist = @(q1, q2) (sum((q2-q1).^2,1).^0.5);

gamma = 15; % can be calculated explicitely, 10 is nice
z=0:pi/24:2*pi;

i=2;
while i<=N+1
    % Generate random points
    if i==N+1
        q_new = goal;
    else
        q_new = field.bound.*rand(param.dim,1);
    end
    nodes(:, i) = q_new;
    % Calculate distances between random new point and points nearby
    [distance, nodes_near, ball_radius]=findNearNodes(nodes, q_new, param.dim, gamma, nodeDist);
    cost_tmp = inf(length(nodes_near),1);
    if ~isempty(nodes_near)
        scatter(nodes(1,i),nodes(2,i))
        text(nodes(1,i),nodes(2,i),num2str(i))
        x=ball_radius*cos(z)+nodes(1,i);
        y=ball_radius*sin(z)+nodes(2,i);
        plot(x,y)
        for t=1:length(nodes_near)
            cost_tmp(t) = cost(nodes_near(t))+ distance(t);
        end
        [cost(i), idx_min_cost] = min(cost_tmp);
        new_edge = [nodes_near(idx_min_cost); i];
        edges = [edges new_edge]; % "connection via indices"
        line([nodes(1, edges(1, i-1)) nodes(1, edges(2, i-1))], ...
            [nodes(2, edges(1, i-1)) nodes(2, edges(2, i-1))], ...
            'Color', 'red')
        
        % Rewire edges if other paths are shorter.
        nodes_near(idx_min_cost)=[];
        distance(idx_min_cost)=[];
        for t=1:length(nodes_near)
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
    else
        if i==N+1
            disp('Cannot connect to goal')
            return
        else
            nodes(:,i)=[0, 0]';
            i=i-1;
        end
    end
    i=i+1;
    disp(['i = ', num2str(i)])
end

for j=1:length(edges)
    line([nodes(1, edges(1, j)) nodes(1, edges(2, j))], ...
         [nodes(2, edges(1, j)) nodes(2, edges(2, j))], ...
         'Color', 'black')
end

% Trace path back from goal to start
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



