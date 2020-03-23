function [shapes]= PlayingFieldV2(amount,size,field_size,verti)
%create random location of obstacles such that the obstacle does not hit
%the goal or end position of drone
x_pos=(field_size(1)-size)*rand(amount,1)+size/2;
y_pos=field_size(2)*rand(amount,1);
z_pos=field_size(3)*rand(amount,1);
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
shapes(u).alpha=shp;
plot(shp)
hold on
end
end