function [coll]=LineInObstacle(point1,point2,shape)
xx=linspace(point1(1),point2(1),100);
yy=linspace(point1(2),point2(2),100);
zz=linspace(point1(3),point2(3),100);
tf = inShape(shape,xx,yy,zz);
coll=any(tf);
end
