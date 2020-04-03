deltaxyz = zeros(length(path)-1,3);
NodeDistance = zeros(length(path)-1);

for i=1:(length(path)-1)
    
    delta_x = nodes(1,path(i+1)) - nodes(1,path(i));
    delta_y = nodes(2,path(i+1)) - nodes(2,path(i));
    delta_z = nodes(3,path(i+1)) - nodes(3,path(i));
    deltaxyz(i,1)= delta_x
    deltaxyz(i,2)= delta_y
    deltaxyz(i,3)= delta_z
    
    NodeDistance(i)=(deltaxyz(i,1).^2+deltaxyz(i,2).^2+deltaxyz(i,3).^2).^0.5;
    TotalDistance = sum (NodeDistance)
    
end

