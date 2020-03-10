close all
field_size=[5,5,5];
e=5;
x_pos=rand(e,1)*field_size(1);
y_pos=rand(e,1)*field_size(2);
z_pos=rand(e,1)*field_size(3);

for u=1:e
n = 12; % Number of vertices
r=0.5;
theta = 2*pi*rand(n,1)-pi; % Random theta
phi = pi*rand(n,1) - pi/2; % Random phi
x = cos(phi).*cos(theta)+x_pos(u); % Create x values
y = cos(phi).*sin(theta)+y_pos(u); % Create y values
z = sin(phi)+z_pos(u); % Create z values
P = [x y z];
P = unique(P,'rows');
shp = alphaShape(P(:,1),P(:,2),P(:,3),1);
shapes(u).alpha=shp;
plot(shp)
hold on
end