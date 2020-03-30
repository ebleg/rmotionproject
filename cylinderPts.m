function [pts] = cylinderPts(R, P1, P2, Nl, Nc)
    % Original points, original plane
    % https://www.mathworks.com/matlabcentral/answers/79296-how-to-do-a-3d-circle-in-matlab
    t = linspace(0,2*pi, Nc);
    circPts = [R*cos(t); R*sin(t); 0*t];
    % unit normal for original plane
    n0 = [0;0;1]; 
    n0 = n0/norm(n0);
    % unit normal for plane to rotate into 
    % plane is orthogonal to n1... given by equation
    % n1(1)*x + n1(2)*y + n1(3)*z = 0
    n1 = P2 - P1; 
    n1 = n1/norm(n1); 
    % theta is the angle between normals
    c = dot(n0,n1)/( norm(n0)*norm(n1) ); % cos(theta)
    s = sqrt(1-c*c);                        % sin(theta)
    u = cross(n0,n1)/( norm(n0)*norm(n1) ); % rotation axis...
    u = u/norm(u); % ... as unit vector
    C = 1-c;
    % the rotation matrix
    Rot = [u(1)^2*C+c, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
           u(2)*u(1)*C+u(3)*s, u(2)^2*C+c, u(2)*u(3)*C-u(1)*s
           u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+c];
    circPts = Rot*circPts;
 
    linePts = [linspace(P1(1), P2(1), Nl); 
               linspace(P1(2), P2(2), Nl); 
               linspace(P1(3), P2(3), Nl)];

    pts = zeros(3, Nl*Nc);
    for i = 1:Nl
        for j = 1:Nc
            pts(:, (i-1)*Nc + j) = circPts(:,j) + linePts(:,i);
        end
    end
    
end

