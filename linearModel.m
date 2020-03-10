%% Linearized 3D-quadcopter model

%% Translational dynamics
A_trans = zeros(6, 6);
A_trans(4:6, 1:3) = eye(3);

% u_trans = [T phi theta g]
B_trans = [0                param.env.g*sin(param.sp.psi0)  param.env.g*cos(param.sp.psi0)   0;
           0                param.env.g*cos(param.sp.psi0)  -param.env.g*sin(param.sp.psi0)  0;
           1/param.drone.m  0                               0                               -1;
           zeros(3, 4)];       
C_trans = [zeros(3), eye(3)];
D_trans = 0;

lti_trans = ss(A_trans, B_trans, C_trans, D_trans);

%% Rotational dynamics
A_rot = zeros(6, 6);
A_rot(4:6, 1:3) = eye(3);
B_rot = [param.drone.I^-1;
         zeros(3, 3)];
C_rot = [zeros(3), eye(3)];
D_rot = 0;

lti_rot = ss(A_rot, B_rot, C_rot, D_rot);

%% Rotational control
Q = eye(6);
R = eye(3);

% Assume full state LQR control 
[K_rot, ~, ~] = lqr(lti_rot, Q, R);
lti_rot_ctrl = feedback(ss(A_rot, B_rot, eye(6), 0), K_rot, -1);

%% Position control 


