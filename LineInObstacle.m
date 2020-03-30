function [coll_node] = LineInObstacle(point2, nodes_near2, shapes2, r)
    n = 60;  %number of points on line
    s3 = 15;   %amount of sphere points
    % nodes_1=nodes2(:,nodes_near2);
    % s=size(nodes_1,2);   %number of loops for checking every nearby point
    s2 = size(shapes2,2);      %number of loops for checking with every shape
    % coll_node=NaN([1,s]);       %empty array for test if line collides with any shape
    % coll_shape=NaN([1,s2]);       %empty array for test if line collides with specific shape

    %Create points on the line
%     xx = linspace(point2(1),nodes_near2(1),n);
%     yy = linspace(point2(2), nodes_near2(2),n);
%     zz = linspace(point2(3), nodes_near2(3),n);
%     
%     [x1, y1, z1] = sphere(s3-1); %create a sphere of points which will be surrounding each point which has been created on the line
%     x1 = reshape(x1,1,[]);        %make from s3xs3 matrix one row matrix
%     y1 = reshape(y1,1,[]);        %make from s3xs3 matrix one row matrix
%     z1 = reshape(z1,1,[]);        %make from s3xs3 matrix one row matrix
%     all_points = NaN(3,s3^2*n);     %create empty array which will fit all the coordinates of the sphere points
%     for e=1:n
%         x1s = r*x1 + xx(e);               %create spherical points surrounding point where drone is
%         y1s = r*y1 + yy(e);               %create spherical points surrounding point where drone is
%         z1s = r*z1 + zz(e);               %create spherical points surrounding point where drone is
%         points_sphere = [x1s;y1s;z1s];   %merge the coordinates of the points
%         all_points(:, (1+s3^2*(e-1)):e*s3^2) = points_sphere;      %add coordinates to array containing all sphere points
%     end
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
%             coll_node_1(1,y) = any(tf);   % check if any of the points collides with shape
            if tf
                collision = true;
            end
            k = k+1;
        end
        y = y+1;
    end
    fprintf("inShape called %d out of %d times\n", k*y, s2*size(all_points, 2))
    
    coll_node = collision;
%     coll_node = any(coll_node_1);   %Check if node connection collides with any shape
end
