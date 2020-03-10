%% Linearized 3D-quadcopter model

%% Translational dynamics
A_trans = zeros(6, 6);
A_trans(4:6, 1:3) = eye(3);

% u_trans = [T phi theta g]
B_trans = [0                param.env.g*sin(param.sp.psi0)  param.env.g*cos(param.sp.psi0)   0;
           0                param.env.g*cos(param.sp.psi0)  -param.env.g*sin(param.sp.psi0)  0;
           1/param.drone.m  0                               0                               -1;
           zeros(3, 4)];       
C_trans = eye(6);
D_trans = zeros(6, 4);

%% Rotational dynamics
A_rot = zeros(6, 6);
A_rot(4:6, 1:3) = eye(3);
B_rot = [param.drone.I^-1;
         zeros(3, 3)];
C_rot = eye(6);
D_rot = zeros(6, 3);

%% Rotational control
Q = eye(6);
R = eye(3);

[K_rot, ~, ~] = lqr(lti_rot, Q, R);


