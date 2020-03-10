%% MAIN
clear;
close all;

field = struct();
field.height = 20;
field.width = 20;
field.center = 0.5*[field.width, field.height]';
field.bound = [field.width, field.height]';

start = [10, 3]';
goal = [10, 15]';

figure
xlim([0 field.width]);
ylim([0 field.height]);
xlabel('x');
ylabel('y');
grid on; grid minor;
axis square

obstacleColor = [0 .5 .5];
rectangle('Position',[0 7 9.5 1], 'FaceColor', obstacleColor)
rectangle('Position',[10.5 7 9.5 1], 'FaceColor', obstacleColor)
rectangle('Position',[5.25 10 9.5 1], 'FaceColor', obstacleColor)
hold on;
scatter(start(1), start(2), 'filled', 'MarkerEdgeColor', [0 0 0]);
scatter(goal(1), goal(2), 'filled', 'MarkerEdgeColor', [0 0 0]);

N = 5*1e3;
nodes = nan(2, N+1);
edges = nan(2, N+1);

nodes(:, 1) = start;
nodeDist = @(q1, q2) sum((q2 - q1).^2, 1);

for i=2:N+1
    q_new 
    field.bound.*rand(2,1);
    [~, I] = min(nodeDist(q_new, nodes));
    edges(:, i) = [I, i]; % "connection via indices"
    nodes(:, i) = q_new;
end

for i=2:N+1
    line([nodes(1, edges(1, i)) nodes(1, edges(2, i))], ...
         [nodes(2, edges(1, i)) nodes(2, edges(2, i))], ...
         'Color', 'black')
end


