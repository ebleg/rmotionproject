function [shp] = polyhedron(maxField, amount)
n = 12; % Number of vertices
r=2;

for i = 1:amount
    theta = 2*pi*rand(n,1)-pi; % Random theta
    phi = pi*rand(n,1) - pi/2; % Random phi
    x = abs(cos(phi).*cos(theta)); % Create x values
    y = abs(cos(phi).*sin(theta)); % Create y values
    z = abs(sin(phi)); % Create z values
%     [x1,y1,z1] = sphere(24);
%     x1=r*x1;
%     y1=r*y1;
%     z1=r*z1;
%     x1 = x1(:);
%     y1 = y1(:);
%     z1 = z1(:);
%     S = [x1 y1 z1];
%     S = unique(S,'rows');
    x1 = abs(x + maxField(1)*rand);
    y1 = abs(y + maxField(2)*rand);
    z1 = abs(z + maxField(3)*rand);
    P = [x1 y1 z1];
    P = unique(P,'rows');
    shp = alphaShape(P(:,1),P(:,2),P(:,3),1);
    hold on;
    plot(shp)
end

% tf = inShape(shp,x1,y1,z1);
% coll=any(tf)

end

