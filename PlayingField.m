function [shapes]= PlayingField(amount,size,field_size,verti, start, goal)

x_pos=rand(amount,1)*field_size(1);
y_pos=rand(amount,1)*field_size(2);
z_pos=rand(amount,1)*field_size(3);
% x_pos=0;
% y_pos=0;
% z_pos=0;

for u=1:amount
    n = verti; % Number of vertices
    theta = 2*pi*rand(n,1)-pi; % Random theta
    phi = pi*rand(n,1) - pi/2; % Random phi
    x = cos(phi).*cos(theta).*size+x_pos(u); % Create x values
    y = cos(phi).*sin(theta).*size+y_pos(u); % Create y values
    z = sin(phi).*size+z_pos(u); % Create z values
    P = [x y z];
    P = unique(P,'rows');
    shp = alphaShape(P(:,1),P(:,2),P(:,3),size);

    tf1 = inShape(shp,start(1), start(2), start(3)); %check if points collide with start
    tf2 = inShape(shp,goal(1), goal(2), goal(3)); %check if points collide with goal
    coll=any(tf1)|| any(tf2);
    %% TODO: Dit is niet helemaal netjes, omdat het aantal obstacles minder wordt 
    % als er eentje in collision is met het start of eindpunt.
    if coll     
        disp('Obstacle collides with goal or start.')
    else
        shapes=shp;
        plot(shp)
        hold on
    end

end

end