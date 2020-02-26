%% EQUATIONS OF MOTION

% gamma = [phi, theta, psi]'
% x = [x, y, z]';

% phi = roll
% theta = pitch
% psi = yaw

omega = convertToOmega(thetadot, theta);
dx = [acc(u, theta, xdot, m, g, k, kd);
      ang_acc(u, omega, I, L, b, k)];

function omega = convertToOmega(gamma, dgamma)
    M = [ 1 0 -sin(gamma(2));
          0 cos(gamma(1)) cos(gamma(2))*sin(gamma(1));
          0 -sin(gamma(1)) cos(gamma(2))*cos(gamma(1))];
    omega = M*dtheta;
end

function rot = rotationMatrix(gamma)
    rot = [cos(gamma(1))*cos(gamma(3)) - cos(gamma(2))*sin(gamma(1))*sin(gamma(3)), -cos(gamma(3))*sin(gamma(1)) - cos(gamma(1))*cos(gamma(2))*sin(gamma(3)), sin(gamma(2))*sin(gamma(3));
           cos(gamma(2))*cos(gamma(3))*sin(gamma(1)) + cos(gamma(1))*sin(gamma(3)), cos(gamma(1))*cos(gamma(2))*cos(gamma(3)) - sin(gamma(1))*sin(gamma(3)), -cos(gamma(3))*sin(gamma(2)); 
           sin(gamma(1))*sin(gamma(2)), cos(gamma(1))*sin(gamma(1)), cos(gamma(2))];
end

function T = thrust(inputs, k)
    T = [0 0 k*sum(inputs)]';
end

function tau = torques(inputs, L, b, k)
    tau = [ L*k*(inputs(1) - inputs(3)); 
            L*k*(inputs(2) - inputs(4));
            b*(inputs(1) - inputs(2) + inputs(3) - inputs(4))];
end

function a = acc(inputs, angles, xdot, m, g, k, kd)
    gravity = [0 0 -g]';
    R = rotation(angles);
    T = R*thrust(inputs, k);
    Fd = -kd*xdot;
    a = gravity + l/m*T + Fd;
end

function domega = ang_acc(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    domega = inv(I) + (tau - cross(omega, I*omega));
end