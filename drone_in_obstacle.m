function [coll]=drone_in_obstacle(x_point,y_point,z_point,shape,radius)
n = 12; % Number of vertices
r=0.5;
theta = 2*pi*rand(n,1)-pi; % Random theta
phi = pi*rand(n,1) - pi/2; % Random phi
x = cos(phi).*cos(theta); % Create x values
y = cos(phi).*sin(theta); % Create y values
z = sin(phi); % Create z values
[x1,y1,z1] = sphere(24);
x1=r*x1+x_point;
y1=r*y1+y_point;
z1=r*z1+z_point;
x1 = x1(:);
y1 = y1(:);
z1 = z1(:);
S = [x1 y1 z1];
S = unique(S,'rows');
P = [x y z];
P = unique(P,'rows');
shp = alphaShape(P(:,1),P(:,2),P(:,3),1);
plot(shp)
axis equal
grid on
hold on
plot(shp2)
axis equal
tf = inShape(shp,x1,y1,z1);
coll=any(tf)
