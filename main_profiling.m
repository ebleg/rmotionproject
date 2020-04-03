%% MAIN WITH VISUALISATION
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
goal = [field.length, 0*field.width, field.height]';


%% Obstacle in 3D
shapes = struct();
shapes(1).alpha = alphaShape([0 0 20 20 0 0 20 20]', [0 15 0 15 0 15 0 15]', [4.5*ones(4,1); 5.5*ones(4,1)]); 
shapes(2).alpha = alphaShape([0 0 20 20 0 0 20 20]', [5 20 5 20 5 20 5 20]', [9.5*ones(4,1); 10.5*ones(4,1)]); 
shapes(3).alpha = alphaShape([0 0 20 20 0 0 20 20]', [0 15 0 15 0 15 0 15]', [14.5*ones(4,1); 15.5*ones(4,1)]);

nodeDist = @(q1, q2) (sum((q2-q1).^2,1).^0.5);

[Nrng, gammarng] = meshgrid(50:25:350, 10:10:60);
solTime = nan(size(Nrng));
pathLengths = nan(size(Nrng));

%% Profiling loop
for iteration = 1:numel(Nrng)
    tic;
    rng(0) % Keep random seed identical
    N = Nrng(iteration);
    gamma = gammarng(iteration);
    fprintf("Iteration %d/%d: N = %d \t gamma = %d\n", iteration, numel(Nrng), N, gamma);
    
    nodes = zeros(param.dim, N+1);
    edges = [];
    path = [];
    cost = inf(1, N+1); % Shortest distance of the current point to start
    failed = false; % flag for failure to connect start and end node

    nodes(:, 1) = start;
    cost(1) = 0; % Cost of the start is zero.

    i=2;
    
    while (i <= N+1) && ~failed
        % Sample random new point in the workspace
        if i == N+1
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
            if ~(nodes_near == i) 
                % For all points within the ball radius, it is checked if the
                % edge between the new point and the other points collides with
                % an obstacle. And the cost for the new point is calculated and
                % the new point is connected to the point that results in the
                % lowest cost for the new point.
                cost_tmp = inf(length(nodes_near),1);
                coll_near_node = true(length(nodes_near),1);
                for t=1:length(nodes_near)
                    coll_near_node(t) = LineInObstacle(q_new,nodes(:,nodes_near(t)),shapes,param.drone.r);
                    if ~coll_near_node(t)
                        cost_tmp(t) = cost(nodes_near(t)) + distance(t);
                    end
                end
                if any(~coll_near_node) % Als iemand iets beters hiervoor weet...
                    [cost(i), idx_min_cost] = min(cost_tmp);                
                    new_edge = [nodes_near(idx_min_cost); i];

                    edges = [edges new_edge]; % "connection via indices"
                    nodes_near(idx_min_cost) = [];
                    coll_near_node(idx_min_cost) = [];
                    distance(idx_min_cost) = [];

                    % Rewire edges from previous points to the new point if that
                    % results in a lower cost for the previous points (this is only
                    % done for the point within the given ball radius)
    %                 coll_near_node = true(length(nodes_near),1);
                    for t=1:length(nodes_near)
    %                   coll_near_node(t) = LineInObstacle(nodes(:,idx_min_cost),nodes(:,nodes_near(t)),shapes,param.drone.r);
                        if ~coll_near_node(t)
                            cost_tmp_1 = cost(i)+ distance(t);
                            if cost_tmp_1 < cost(nodes_near(t))
                                edges_del = find(edges(2,:)== nodes_near(t));
                                edges(:,edges_del) = [];

                                new_edge = [i; nodes_near(t)];
                                ind = nodes_near(t);
                                edges = [edges new_edge]; % "connection via indices"
                            end
                        end
                    end         
                else
                    if i==N+1
                        disp('Cannot connect to goal');
                        failed = true;
                    else
                        nodes(:,i)=[0, 0, 0]';
                        i=i-1;
                    end
                end
            else
                if i==N+1
                    disp('Cannot connect to goal');
                    failed = true;
                else
                    nodes(:,i)=[0, 0, 0]';
                    i=i-1;
                end
            end
        else
            nodes(:,i)=[0, 0, 0]';
            i=i-1;
        end
        i=i+1;
%         disp(['i = ', num2str(i)])
    end
    
    % Trace back from goal (edges all have unique parent)
    if ~failed
        found = false;
        node = N+1;
        path = [node];
        totalLength = 0;
        while ~found
            newNode = edges(1, edges(2,:) == node);
            totalLength = totalLength + nodeDist(nodes(:,node), nodes(:,newNode));
            node = newNode;
            path = [path, node];
            if node == 1
                found = true;
            end
        end
        pathLengths(iteration) = totalLength;
    end
    solTime(iteration) = toc;   
    
    %%
    close all;
    figure('units','normalized','outerposition',[0 0 1 1], 'name', strcat("N=",string(N)," ","gamma=",string(gamma)))
    plot3(field.length, field.width, field.height)
    xlim([0 field.length]);
    ylim([0 field.width]);
    zlim([0 field.height]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title("RRT* path planning");
    grid on; grid minor; hold on;
    axis equal;
    cla;
    for k=1:param.obs.amount
        plot(shapes(k).alpha, 'FaceAlpha', 0.7, 'EdgeAlpha', 1) % plot obstacles;
    end
    scatter3(start(1), start(2), start(3), 'filled', 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'cyan');
    text(start(1), start(2), start(3), '  Start node')
    scatter3(goal(1), goal(2), goal(3), 'filled', 'MarkerEdgeColor', [0 0 0], 'MarkerFaceColor', 'red');
    text(goal(1), goal(2), goal(3), '  Target node')
    
    for k=1:N+1
        scatter3(nodes(1,k),nodes(2,k),nodes(3,k), 'o', 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'black')
        text(nodes(1,k)+0.8,nodes(2,k)+0.8,nodes(3,k)+0.8,num2str(k))
    end
    for j=1:length(edges)
        line([nodes(1, edges(1, j)) nodes(1, edges(2, j))], ...
            [nodes(2, edges(1, j)) nodes(2, edges(2, j))], ...
            [nodes(3, edges(1, j)) nodes(3, edges(2, j))], ...
            'Color', 'black')
    end
    title(strcat("RRT* path planning: i=", string(i), '/', string(N+1)), 'Fontsize', 18);
    for j=2:length(path)
        line([nodes(1, path(j-1)) nodes(1, path(j))], ...
            [nodes(2, path(j-1)) nodes(2, path(j))], ...
            [nodes(3, path(j-1)) nodes(3, path(j))], ...
            'Color', 'cyan', 'Linewidth', 2)
    end
    pause(0.01);
    
end
