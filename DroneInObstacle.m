function [coll]=DroneInObstacle(point,shape,r)
[x1,y1,z1] = sphere(24);
x1=r*x1+point(1);
y1=r*y1+point(2);
z1=r*z1+point(3);
tf = inShape(shape,x1,y1,z1);
coll=any(tf);
end
