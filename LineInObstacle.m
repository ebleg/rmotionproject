function [coll_node]=LineInObstacle(point2,nodes_near2,nodes2,shapes2)
n=20;  %number of points on line
nodes_1=nodes2(:,nodes_near2);
s=size(nodes_1,2);   %number of loops for checking every nearby point
s2=size(shapes2,2);      %number of loops for checking with every shape
coll_node=NaN([1,s]);       %empty array for test if line collides with any shape
coll_shape=NaN([1,s2]);       %empty array for test if line collides with specific shape

for u=1:s
%Create points on the line
xx=linspace(point2(1),nodes_1(1,u),n);
yy=linspace(point2(2),nodes_1(2,u),n);
zz=linspace(point2(3),nodes_1(3,u),n);
for y=1:s2
tf = inShape(shapes2(y).alpha,xx,yy,zz); %check if line of node collides with one of the shapes
coll_shape(y)=any(tf);     %Check if line collides with specific of the shapes
end
coll_node(u)=any(coll_shape);   %Check if node connection collides with any shape
end
end
