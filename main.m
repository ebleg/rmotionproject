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

start = [10, 10, 3]';
goal = [10, 10, 15]';
dim = 3;        % Dimension of the workspace
amount = 15;     % Amount of obstacles in the workspace

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
[shapes]= PlayingField(param.obs.amount,param.obs.size,[field.length field.width field.height],param.obs.verti,  start, goal);
hold on;
scatter3(start(1), start(2), start(3), 'filled', 'MarkerEdgeColor', [0 0 0]);
scatter3(goal(1), goal(2), goal(3), 'filled', 'MarkerEdgeColor', [0 0 0]);

N = 5*1e3;
nodes = zeros(dim, N+1);
edges = zeros(2, N+1);

nodes(:, 1) = start;
nodeDist = @(q1, q2) sum((q2 - q1).^2, 1);

gamma = 3; % can be calculated explicitely

for i=2:N+1
    q_new = field.bound.*rand(dim,1);
    [~, I] = min(nodeDist(q_new, nodes));
    nodes(:, i) = q_new;
    % check if q_new is in collision
    coll = DroneInObstacle(q_new,shapes,param.drone.r);
%     if ~coll
        % Optimal RRT
        [nodes_near]=findNearNodes(nodes, q_new, dim, gamma, nodeDist);
        edges(:, i) = [I, i]; % "connection via indices"
        nodes(:, i) = q_new;
%     end     
end

for i=2:N+1
    line([nodes(1, edges(1, i)) nodes(1, edges(2, i))], ...
         [nodes(2, edges(1, i)) nodes(2, edges(2, i))], ...
         [nodes(3, edges(1, i)) nodes(3, edges(2, i))], ...
         'Color', 'black')
end