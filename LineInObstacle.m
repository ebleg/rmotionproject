function [coll_node] = LineInObstacle(point2, nodes_near2, shapes2, r)
    n = 30;  %number of points on line
    s3 = 10;   %amount of sphere points
    s2 = size(shapes2,2);      %number of loops for checking with every shape

    if all(point2 == nodes_near2)
       all_points = point2; 
    else
        all_points = cylinderPts(r, point2, nodes_near2, n, s3);
    end
%     coll_node_1 = zeros(1,s2);
    y = 1;
    collision = false;
    while ~collision && (y <= s2)  % for all shapes
        k = 1;
        while ~collision && (k <= size(all_points, 2))
            tf = inShape(shapes2(y).alpha, all_points(:,k)'); % check if line of node collides with one of the shapes
            if tf
                collision = true;
            end
            k = k+1;
        end
        y = y+1;
    end
%     fprintf("inShape called %d out of %d times\n", k*y, s2*size(all_points, 2))
    
    coll_node = collision;
end
