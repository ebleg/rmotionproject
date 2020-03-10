function [coll]=LineInObstacle(point1,point2,shape)
n=100;  %number of points on line
%Create points on the line
xx=linspace(point1(1),point2(1),n);
yy=linspace(point1(2),point2(2),n);
zz=linspace(point1(3),point2(3),n);
tf = inShape(shape,xx,yy,zz); %check if points collide with obstacle
coll=any(tf);
end
