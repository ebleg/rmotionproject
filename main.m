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
[shapes]= PlayingFieldV2(param.obs.amount,param.obs.size,[field.length field.width field.height],param.obs.verti);
hold on;
scatter3(start(1), start(2), start(3), 'filled', 'MarkerEdgeColor', [0 0 0]);
scatter3(goal(1), goal(2), goal(3), 'filled', 'MarkerEdgeColor', [0 0 0]);

N = 5e2; 
nodes = zeros(param.dim, N+1);
edges = zeros(2, 1);

nodes(:, 1) = start;
nodeDist = @(q1, q2) ((sum(q2-q1,1).^2).^0.5);

gamma = 60; % can be calculated explicitely

for i=2:N+1
    i
    q_new = field.bound.*rand(param.dim,1);
    nodes(:, i) = q_new;
    coll_point = DroneInObstacle(q_new,shapes,param.drone.r);   %check if point is a valaible position for the drone
    if coll_point==0
        [nodes_near]=findNearNodes(nodes, q_new, param.dim, gamma, nodeDist);
        coll_near_node=LineInObstacle(q_new,nodes_near,nodes,shapes);    %Check for every noded if the route to that node collides woth obstacle
        q=size(coll_near_node,2);
        for o=1:q
        if coll_near_node(o)==0
            new_edge = [nodes_near(o); i];
            edges = [edges new_edge]; % "connection via indices"
        end
        end
    end
end

% scatter3(nodes(1,:),nodes(2,:),nodes(3,:))

for u=2:length(edges)
    line([nodes(1, edges(1, u)) nodes(1, edges(2, u))], ...
         [nodes(2, edges(1, u)) nodes(2, edges(2, u))], ...
         [nodes(3, edges(1, u)) nodes(3, edges(2, u))], ...
         'Color', 'black')
end