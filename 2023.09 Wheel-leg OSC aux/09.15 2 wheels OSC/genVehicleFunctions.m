% genFunctions.m
clc; clear; close all

mkdir('auto')
addpath(genpath( 'auto' ) );
addpath(genpath( 'Functions' ) );


cross_op = @(x) ...
    [0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];

% syms t q1(t) q2(t) q_1 q_2 dq_1 dq_2 ddq_1 ddq_2 tau_1 tau_2

params = getVehicleParams();
DOF = 15; % DOF = 11;
Nact = 6;

slope_angle = 0; %-pi/4;
r_slope = [cos(slope_angle); sin(slope_angle)];

syms t

% Note: 
% Symbolic functions q1(t), q2(t), ... and their derivatives are used to 
% compute E_L_eq.
% E_L_eq is then updated. Symbolic functions q1(t), q2(t), ... and their 
% derivatives are substituted with symbolic variables q_1, q_2, ...
% Not sure if/how the symbolic arrays q, q_, dq_, ddq_ are used.


% Create symbolic functions 
for iter = 1:DOF 
    syms(['q' num2str(iter) '(t)'])
end
% Create a symbolic array q = [q1(t); q2(t); ... ].
for iter = 1:DOF
    q(iter,1) = eval(['q' num2str(iter)]);
    q_char{iter,1} = eval(['q' num2str(iter)]);
end

syms('q_',[DOF 1])
syms('dq_',[DOF 1])
syms('ddq_',[DOF 1])
syms('u_',[Nact 1])


%% Build Kinematics
% Base position
% r_c(1:2,1) = [q(1); q(2)];
% theta_c(1,1) = q3;

% Joint locations
ij = 0;
% Base joint (1)
ij = ij+1;
r_j(1:2,ij) = [q(1); q(2)]; % Euclidean vector. q(1) = q1(t).
t_j(1,ij) = q3; % Planar rotation

% Front leg joint (2)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-1) + params.L1a*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-1)+q4; % Planar rotation

% Front knee joint (3)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-1) + params.L2*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-1)+q5; % Planar rotation

% Front wheel joint A (4)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-1) + params.L3a*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-1)+q6; % Planar rotation

% Front wheel joint B (5)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-2) - params.L3b*[cos(t_j(1,ij-2));sin(t_j(1,ij-2))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-2)+q7; % Planar rotation


% Back leg joint (6)
ij = ij+1;
r_j(1:2,ij) = r_j(:,1) - params.L1b*[cos(t_j(1,1));sin(t_j(1,1))]; % Euclidean vector
t_j(1,ij) = t_j(1,1)+q8; % Planar rotation

% Back knee joint (7)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-1) + params.L6*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-1)+q9; % Planar rotation

% Back wheel joint A (8)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-1) + params.L7a*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-1)+q10; % Planar rotation

% Back wheel joint B (9)
ij = ij+1;
r_j(1:2,ij) = r_j(:,ij-2) - params.L7b*[cos(t_j(1,ij-2));sin(t_j(1,ij-2))]; % Euclidean vector
t_j(1,ij) = t_j(1,ij-2)+q11; % Planar rotation

% Center Points

ij = 0;
% Base center (1)
ij = ij+1;
r_c(1:2,ij) = r_j(1:2,ij);

% Front thigh center (2)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij) + params.L2/2*[cos(t_j(1,ij));sin(t_j(1,ij))];

% Front shin center (3)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij);

% Wheel A center (4)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij-1) + params.L3a*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))];

% Wheel B center (5)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij-2) - params.L3b*[cos(t_j(1,ij-2));sin(t_j(1,ij-2))];


% Back thigh center (6)
ij = ij+1;
r_c(1:2,ij) = r_j(:,6) + params.L6/2*[cos(t_j(1,ij));sin(t_j(1,ij))];

% Back shin center (7)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij);

% Back A center (8)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij-1) + params.L7a*[cos(t_j(1,ij-1));sin(t_j(1,ij-1))];

% Back B center (9)
ij = ij+1;
r_c(1:2,ij) = r_j(:,ij-2) - params.L7b*[cos(t_j(1,ij-2));sin(t_j(1,ij-2))];

%% Setup for constraints. Parametric functions for the wheels.

% ------------- fwA
phi = q(12) ;   dphi = diff(q(12)) ;    theta = t_j(1,4) ;

% wheel_radius = 0.75 ; 
alpha_wheel = params.wheel_radius* [cos(phi); sin(phi)] ; % Parametric function of a circle.
d_alpha_wheel_dphi = diff(alpha_wheel, phi) ;

% Calculate the norm explicitly because the 'norm' function introduces abs(p), which can be problematic.
norm_d_alpha_wheel_dphi = sqrt(d_alpha_wheel_dphi(1,1)^2 + d_alpha_wheel_dphi(2,1)^2) ;
norm_d_alpha_wheel_dphi = expand(norm_d_alpha_wheel_dphi) ;

T_wheel = simplify( 1/norm_d_alpha_wheel_dphi * d_alpha_wheel_dphi ) ; % Tangent vector.
% s_wheel = int(norm_d_alpha_wheel_dphi, phi, 0, phi)  % Arc-length.
% d_s_wheel_dt = fulldiff(s_wheel, q_char) 
d_s_wheel_dt = simplify( norm_d_alpha_wheel_dphi * dphi ) ;

% ------------- bwB
phi_bwB = q(14) ;   
dphi_bwB = diff(q(14)) ;    
theta_bwB = t_j(1,9) ;

alpha_bwB = params.wheel_radius* [cos(phi_bwB); sin(phi_bwB)] ; % Parametric function of a circle.
d_alpha_dphi_bwB = diff(alpha_bwB, phi_bwB) ;

% Calculate the norm explicitly because the 'norm' function introduces abs(p), which can be problematic.
norm_d_alpha_dphi_bwB = sqrt(d_alpha_dphi_bwB(1,1)^2 + d_alpha_dphi_bwB(2,1)^2) ;
norm_d_alpha_dphi_bwB = expand(norm_d_alpha_dphi_bwB) ;

T_bwB = simplify( 1/norm_d_alpha_dphi_bwB * d_alpha_dphi_bwB ) ; % Tangent vector.
% s_wheel = int(norm_d_alpha_wheel_dphi, phi, 0, phi)  
% d_s_wheel_dt = fulldiff(s_wheel, q_char) 
% Time-derivative of arc-length using the chain rule.
d_s_bwB_dt = simplify( norm_d_alpha_dphi_bwB * dphi_bwB ) ;


%% Setup for constraints. Parametric functions for the terrain.

% ------------- fwA contact point
p = q(13) ;     dp = diff(q(13)) ;

% alpha_terr = [p; -0.4142] ; % Parametric function of a line.
% alpha_terr = params.terrain_radius * [cos(p); sin(p)] ; % Parametric function of a circle.
% alpha_terr = [p; sin(p)] ; % Parametric function of a sinusoid.
alpha_terr = terrain_cos(p) ;
d_alpha_terr_dp = diff(alpha_terr, p) ;

% Calculate the norm explicitly because the 'norm' function introduces abs(p), which can be problematic.
norm_d_alpha_terr_dp = sqrt(d_alpha_terr_dp(1,1)^2 + d_alpha_terr_dp(2,1)^2) ;
norm_d_alpha_terr_dp = expand(norm_d_alpha_terr_dp) ;

T_terr = simplify( 1/norm_d_alpha_terr_dp * d_alpha_terr_dp ) ; % Tangent vector.
N_terr = diff(T_terr, p) ; % Normal vector.
norm_N_terr = expand( sqrt(N_terr(1,1)^2 + N_terr(2,1)^2) );
N_unit_terr = N_terr / norm_N_terr ;
% s_terr = int(norm_d_alpha_terr_dp, p, 0, p) ; % Arc-length.
d_s_terr_dt = norm_d_alpha_terr_dp * dp ;

% ------------- bwB contact point
p_bwB = q(15) ;     dp_bwB = diff(q(15)) ;

alpha_t_bwB = terrain_cos(p_bwB) ; % [p_bwB; sin(p_bwB)] ; % Parametric function of a sinusoid.
d_alpha_t_dp_bwB = diff(alpha_t_bwB, p_bwB) ;

% Calculate the norm explicitly because the 'norm' function introduces abs(p), which can be problematic.
norm_d_alpha_t_dp_bwB = sqrt(d_alpha_t_dp_bwB(1,1)^2 + d_alpha_t_dp_bwB(2,1)^2) ;
norm_d_alpha_t_dp_bwB = expand(norm_d_alpha_t_dp_bwB) ;

T_t_bwB = simplify( 1/norm_d_alpha_t_dp_bwB * d_alpha_t_dp_bwB ) ; % Tangent vector.
% N_t_bwB = diff(T_t_bwB, p_bwB) ; % Normal vector.

d_s_t_bwB_dt = norm_d_alpha_t_dp_bwB * dp_bwB ;


%% Substitutions. 
% Cannot use matlabFunction with symbolic functions. Need symbolic
% variables.

alpha_wheel_sub = subs(alpha_wheel, q12(t), q_12) ;
T_wheel_sub = subs(T_wheel, q12(t), q_12) ;
alpha_terr_sub = subs(alpha_terr, q13(t), q_13) ;
T_terr_sub = subs(T_terr, q13(t), q_13) ;
N_unit_terr_sub = subs(N_unit_terr, q13(t), q_13) ;

alpha_bwB_sub = subs(alpha_bwB, q14(t), q_14) ;
T_bwB_sub = subs(T_bwB, q14(t), q_14) ;
alpha_t_bwB_sub = subs(alpha_t_bwB, q15(t), q_15) ;
T_t_bwB_sub = subs(T_t_bwB, q15(t), q_15) ;


%% Constraints using auxiliary coordinates.

% fwA. Position level constraints
h1 = NaN ; % To implement the arc-length constraint at the velocity level.

h2 = dot( RM_CCW(theta)*T_wheel , T_terr ) - 1 ; % Tangent constraint.
h2 = simplify (h2) ; % To simplify using trig identities.

h3 = [r_c(1:2,4)] - alpha_terr + RM_CCW(theta)*alpha_wheel ; % Non-penetration constraint.
h3 = simplify(h3) ;

% fwA. Velocity level constraints
dh1_dt = d_s_wheel_dt - d_s_terr_dt ;      dh1_dt = expand(dh1_dt) ;

% ----------------------------------------------------
% bwB. Position level constraints
h13 = NaN ; % To implement the arc-length constraint at the velocity level.

h14 = dot( RM_CCW(theta_bwB)*T_bwB , T_t_bwB ) - 1 ; % Tangent constraint.
h14 = simplify (h14) ; % To simplify using trig identities.

h15 = [r_c(1:2,9)] - alpha_t_bwB + RM_CCW(theta_bwB)*alpha_bwB ; % Non-penetration constraint.
h15 = simplify(h15) ;

% bwB. Velocity level constraints
dh13_dt = d_s_bwB_dt - d_s_t_bwB_dt ;      dh13_dt = expand(dh13_dt) ;

% ----------------------------------------------------
h = [h1; h2; h3; h13; h14; h15] ;

dh_dt = diff(h, t) ; dh_dt = simplify(dh_dt) ;

% Replace NaN with the velocity level constraints.
dh_dt(1) = dh1_dt ; 
dh_dt(5) = dh13_dt ; 


d2h_dt2 = diff(dh_dt, t) ; d2h_dt2 = simplify(d2h_dt2) ;

con_funcs = dh_dt ;
d_con_funcs = d2h_dt2 ;

%% Constraints. CH's formulation.

% wheel_axis = [0;0;1];
% % Constraint function, front wheel A
% r_wheel = params.L4*cross_op([r_slope; 0])*wheel_axis; % Unit vector normal to the terrain.
% % v = omega X r where v, omega, and r are vectors.
% % omega = angular velocity vector = cross_op(diff(t_j(1,4),t)*[0;0;1]) 
% %       where t_j(1,4) = angular position of the wheel.
% % Variable wxr represents v.
% wxr = cross_op(diff(t_j(1,4),t)*[0;0;1])*r_wheel; 
% fc_fwA = diff(r_j(1:2,4),t) + wxr(1:2,1); % Seems like v - (omega X r) = 0. 

% % Constraint function, front wheel B
% r_wheel = params.L5*cross_op([r_slope; 0])*wheel_axis;
% wxr = cross_op(diff(t_j(1,5),t)*[0;0;1])*r_wheel;
% fc_fwB = diff(r_j(1:2,5),t) + wxr(1:2,1);


% % Constraint function, back wheel A
% r_wheel = params.L8*cross_op([r_slope; 0])*wheel_axis;
% wxr = cross_op(diff(t_j(1,8),t)*[0;0;1])*r_wheel;
% fc_bwA = diff(r_j(1:2,8),t) + wxr(1:2,1);


% % Constraint function, back wheel B
% r_wheel = params.L9*cross_op([r_slope; 0])*wheel_axis;
% wxr = cross_op(diff(t_j(1,9),t)*[0;0;1])*r_wheel;
% fc_bwB = diff(r_j(1:2,9),t) + wxr(1:2,1);


% % Front wheels chained together
% fc_fchain = diff(t_j(1,4)-t_j(1,5),t);

% % Back wheels chained together
% fc_bchain = diff(t_j(1,8)-t_j(1,9),t);

%% Create the constraints vector.

% % con_funcs = [fc_fwA; fc_fwB; fc_bwA; fc_bwB; fc_fchain; fc_bchain];
% con_funcs = [fc_fwA]; % Velocity level.
% % con_funcs = [fc_bwA; fc_fchain; fc_bchain];
% d_con_funcs = diff(con_funcs,t); % Acceleration level.

Nlam = numel(con_funcs);
syms('lam_',[Nlam 1])

%% Build Inertias
m_vec(1,1) = params.m1;
m_vec(1,2) = params.m2;
m_vec(1,3) = params.m3;
m_vec(1,4) = params.m4;
m_vec(1,5) = params.m5;
m_vec(1,6) = params.m6;
m_vec(1,7) = params.m7;
m_vec(1,8) = params.m8;
m_vec(1,9) = params.m9;

I_vec(1,1) = params.I1;
I_vec(1,2) = params.I2;
I_vec(1,3) = params.I3;
I_vec(1,4) = params.I4;
I_vec(1,5) = params.I5;
I_vec(1,6) = params.I6;
I_vec(1,7) = params.I7;
I_vec(1,8) = params.I8;
I_vec(1,9) = params.I9;


%% Energies

V = sum(m_vec.*r_c(2,:)*params.g,2);

dr_dt = diff(r_c,t);
dt_j_dt = diff(t_j,t);
T = sum(1/2*m_vec.*sum(dr_dt.*dr_dt,1) + 1/2*I_vec.*dt_j_dt.*dt_j_dt,2);

L = T - V;


for iter = 1:DOF
    dL_dq(iter,1) = diff(L,eval(['q' num2str(iter) ,'(t)']));
end

for iter = 1:DOF
    dL_ddq(iter,1) = diff(L,eval(['diff(q' num2str(iter) ,'(t),t)']));
end

dL_ddq_dt = diff(dL_ddq,t);

E_L_eq = dL_ddq_dt-dL_dq;


%% TASKS

% Task space coordinates
x_task = [r_c(1:2,1); t_j(1,1)];
% x_task = [r_c(1:2,1); t_j(1,1); r_c(2,4); r_c(2,5); r_c(2,9)];
dxdt_task = diff(x_task,t);



%% SUBSTITUTIONS

% Substitution Loop
for iter = 1:DOF
    E_L_eq = subs(E_L_eq, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
    con_funcs = subs(con_funcs, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
    d_con_funcs = subs(d_con_funcs, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
    
    x_task = subs(x_task, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
    dxdt_task = subs(dxdt_task, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
    
    r_j = subs(r_j, ...
        {eval(['q',num2str(iter),'(t)'])}, ...
        {q_(iter)});
    r_c = subs(r_c, ...
        {eval(['q',num2str(iter),'(t)'])}, ...
        {q_(iter)});
end

J_task = jacobian(x_task,q_);
J_task_temp = J_task;
for iter = 1:DOF
    J_task_temp = subs(J_task_temp,q_(iter),eval(['q',num2str(iter),'(t)']));
end
dJdt_task = diff(J_task_temp,t);
for iter = 1:DOF
    dJdt_task = subs(dJdt_task, ...
        {eval(['diff(q',num2str(iter),'(t),t,t)']), ...
        eval(['diff(q',num2str(iter),'(t),t)']), ...
        eval(['q',num2str(iter),'(t)'])}, ...
        {ddq_(iter), dq_(iter), q_(iter)});
end



    
CoM = sum(([1;1]*m_vec).*r_c,2)/sum(m_vec);
C_term = -subs(E_L_eq,ddq_,zeros(size(ddq_)));

M = jacobian(E_L_eq,ddq_);


A = jacobian(con_funcs,dq_);

f_con_term = A.'*lam_;

% f_noact = -subs(E_L_eq,ddq_,zeros(size(ddq_)));
B = [...
     0, 0, 0, 0, 0, 0; % x
     0, 0, 0, 0, 0, 0; % z
    -1, 0, 0,-1, 0, 0; % t1
     1,-1, 0, 0, 0, 0; % t2
     0, 1,-1, 0, 0, 0; % t3
     0, 0, 1, 0, 0, 0; % t4
     0, 0, 0, 0, 0, 0; % t5
     0, 0, 0, 1,-1, 0; % t6
     0, 0, 0, 0, 1,-1; % t7
     0, 0, 0, 0, 0, 0; % t8
     0, 0, 0, 0, 0, 1; % t9
     0, 0, 0, 0, 0, 0; % phi
     0, 0, 0, 0, 0, 0; % p
     0, 0, 0, 0, 0, 0; % phi_bwB
     0, 0, 0, 0, 0, 0 ... % p_bwB
     ];

% B = [...
%      0; % x
%      0; % z
%      0; % t1
%      0; % t2
%      0; % t3
%      1; % t4
%      0; % t5
%      0; % t6
%      0; % t7
%      0; % t8
%      1; % t9
%      0; % phi
%      0; % p
%      0; % phi_bwB
%      0 ... % p_bwB
%      ] ; 

% B = [...
%      0, 0; % x
%      0, 0; % z
%      0, 0; % t1
%      0, 0; % t2
%      0, 0; % t3
%      1, 0; % t4 fwA
%      0, 0; % t5
%      0, 0; % t6
%      0, 0; % t7
%      0, 0; % t8
%      0, 1; % t9 bwB
%      0, 0; % phi
%      0, 0; % p
%      0, 0; % phi_bwB
%      0, 0 ... % p_bwB
%      ] ; 


Wnc = B*u_;
% J_con = jacobian(con_funcs,ddq_)

% Add actuation
% f = f_noact+Wnc;
f = C_term+Wnc;

E_L_aug = E_L_eq - f_con_term - Wnc;
ddq_aug = [ddq_; lam_];

M_aug = jacobian([E_L_aug; d_con_funcs],ddq_aug);
% M_aug = [M, A.';  jacobian(d_con_funcs,ddq_), zeros(Nlam)];
f_aug = -subs([E_L_aug; d_con_funcs],ddq_aug,zeros(size(ddq_aug)));



%% QP Matrices

% desired task-space acceleration
ddx_des = sym('ddx_des',size(x_task));

% optimization design variables
u_opt = sym('u_opt',size(u_));
lam_opt = sym('lam_opt',size(lam_));
ddq_opt = sym('ddq_opt',size(ddq_));

% Construct design vector
z_opt = [u_opt; ddq_opt; lam_opt]; % Design vector

% Actual acceleration in task space
ddx_act = dJdt_task*dq_ + J_task*ddq_opt;

% Q matrix defining weights in objective function
Q_opt = eye(numel(ddx_act));

% Objective function
J_opt = (ddx_des-ddx_act).'*Q_opt*(ddx_des-ddx_act);

% Hessian of objective function
H_opt = hessian(J_opt, z_opt);
% Linear terms of objective function
f_opt = subs(gradient(J_opt,z_opt), z_opt, zeros(size(z_opt)));

% Equality constriants
% Enforces equations of motion
% Substitutes design variable ddqs and u
EQ_CON = subs([E_L_aug; d_con_funcs],[ddq_; u_; lam_],[ddq_opt; u_opt; lam_opt]);

% Jacobian of equality constraints
A_eq_con = jacobian(EQ_CON, z_opt);
% Constant terms of equality constraints
b_eq_con = -subs(EQ_CON, [ddq_opt; u_opt; lam_opt], [zeros(size(ddq_opt)); zeros(size(u_opt)); zeros(size(lam_opt))]);




matlabFunction(M,'vars',{q_},'file',['auto' filesep 'M_func']);
matlabFunction(f,'vars',{q_,dq_,u_},'file',['auto' filesep 'f_func']);
matlabFunction(r_j,'vars',{q_},'file',['auto' filesep 'rj_func']);
matlabFunction(r_c,'vars',{q_},'file',['auto' filesep 'rc_func']);
matlabFunction(CoM,'vars',{q_},'file',['auto' filesep 'CoM_func']);

% Export
matlabFunction(M_aug,'vars',{q_},'file',['auto' filesep 'M_aug_func']);
matlabFunction(f_aug,'vars',{q_,dq_,u_},'file',['auto' filesep 'f_aug_func']);

matlabFunction(x_task,'vars',{q_},'file',['auto' filesep 'xTask_func']);
matlabFunction(dxdt_task,'vars',{q_,dq_},'file',['auto' filesep 'dxdtTask_func']);
matlabFunction(J_task,'vars',{q_},'file',['auto' filesep 'JTask_func']);
matlabFunction(dJdt_task,'vars',{q_,dq_},'file',['auto' filesep 'dJdtTask_func']);

matlabFunction(H_opt,'vars',{q_,dq_,ddx_des},'file',['auto' filesep 'H_opt_func']);
matlabFunction(f_opt,'vars',{q_,dq_,ddx_des},'file',['auto' filesep 'f_opt_func']);
matlabFunction(A_eq_con,'vars',{q_,dq_},'file',['auto' filesep 'A_eq_func']);
matlabFunction(b_eq_con,'vars',{q_,dq_},'file',['auto' filesep 'b_eq_func']);
matlabFunction(J_opt,'vars',{z_opt,q_,dq_,ddx_des},'file',['auto' filesep 'J_obj_func']);

matlabFunction(alpha_wheel_sub,'vars',{q_12},'file',['auto' filesep 'alpha_wheel_func']);
matlabFunction(T_wheel_sub,'vars',{q_12},'file',['auto' filesep 'T_wheel_func']);
matlabFunction(alpha_terr_sub,'vars',{q_13},'file',['auto' filesep 'alpha_terr_func']);
matlabFunction(T_terr_sub,'vars',{q_13},'file',['auto' filesep 'T_terr_func']);
matlabFunction(N_unit_terr_sub,'vars',{q_13},'file',['auto' filesep 'N_unit_terr_func']);

matlabFunction(alpha_bwB_sub,'vars',{q_14},'file',['auto' filesep 'alpha_bwB_func']);
matlabFunction(T_bwB_sub,'vars',{q_14},'file',['auto' filesep 'T_bwB_func']);
matlabFunction(alpha_t_bwB_sub,'vars',{q_15},'file',['auto' filesep 'alpha_t_bwB_func']);
matlabFunction(T_t_bwB_sub,'vars',{q_15},'file',['auto' filesep 'T_t_bwB_func']);

% matlabFunction(B,'vars',{q_},'file',['auto' filesep 'B_func']);














