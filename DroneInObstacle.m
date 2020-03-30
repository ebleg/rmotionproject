function [coll]=DroneInObstacle(point,shapes,r)
    s=size(shapes,2);       %determine amount of shapes in field
    coll1=NaN([s,1]);       %empty array to fill with test if collision between point and note
    for u=1:s
        [x1, y1, z1] = sphere(24);        %create spherical points surrounding point where drone is
        x1=r*x1 + point(1);               %create spherical points surrounding point where drone is
        y1=r*y1 + point(2);               %create spherical points surrounding point where drone is
        z1=r*z1 + point(3);               %create spherical points surrounding point where drone is
        IS = inShape(shapes(u).alpha,x1,y1,z1); %check if obstacle collide with points from sphere
        coll1(u)=any(any(IS));          %If any of the points collide with obstacle return 1
    end
    coll=any(coll1); %If any of the points collide with obstacles return 1
end
