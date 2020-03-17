function [coll]=DroneInObstacle(point,shapes,r)
s=size(shapes,2);
coll1=NaN([s,1]);
for u=1:s
    [x1,y1,z1] = sphere(24);
    x1=r*x1+point(1);
    y1=r*y1+point(2);
    z1=r*z1+point(3);
    IS = inShape(shapes(u).alpha,x1,y1,z1);
    coll1(u)=any(any(IS));
end
coll=any(coll1);
end
