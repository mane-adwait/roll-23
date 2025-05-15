function tau = GetTorqueOSC(x_des,dx_des,q,dq)

nX = numel(x_des);
nU = 3;

% PD gain matrices
kp = 50*eye(nX); % proportional gains
% kp = 0*eye(nX); % proportional gains

% DERIVATIVE ONLY GAINS FOR JOINTS (Task 4-7)
% kp(4:7,4:7) = 0;

kd = 10*eye(nX); % derivative gains
% kd = 0*eye(nX); % derivative gains



% Torque limits
tau_limit = 50;

x = xTask_func(q); % position in task space
J = JTask_func(q); % Jacobian in task space
dxdt = J*dq; % velocity in task space

% Task space PD control law
ddx_des = kp*(x_des-x)+kd*(dx_des-dxdt);

% Quadratic programming matrices
H = H_opt_func(q,dq,ddx_des); % Hessian
f = f_opt_func(q,dq,ddx_des); % Linear terms
Aeq = A_eq_func(q,dq); % Equality constraint Jacobian
beq = b_eq_func(q,dq); % Equality constraint constant term

nDesignVars = numel(f);
lb = -inf(nDesignVars,1);
ub = inf(nDesignVars,1);

% Bounds on Torques
lb(1:nU) = -tau_limit;
ub(1:nU) = tau_limit;

%% Quadratic Program (QP)
% Display options: 'off', 'final', 'iter', 'iter-detailed',
% 'final-detailed'
% https://www.mathworks.com/help/optim/ug/quadprog.html#btm48d1
options =  optimoptions(@quadprog,'Display','iter');
% Run QP solver
% Design vector z is ordered [u1 u2 ddq1 ddq2].'

z_0 = ones(nDesignVars,1);
[z, ~] = quadprog(H,f,[],[],Aeq,beq,lb,ub,z_0,options);
% [z, ~] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],options);
disp(z)

% Pull optimal torques out of the design vector
% z_temp = z; % copies design vector
% z_temp(end-numel(q)+1:end) = []; % deletes ddqs
tau = z(1:nU); % leaving torques as the remaining design variables