function [dy] = dynVehicleControl(t,y,tau)


q = y(1:2:end);
dq = y(2:2:end);


% tau = GetTorqueOSC([8;1;0.0],[0;0;0],q,dq);

M_aug_out = M_aug_func(q);
f_aug_out = f_aug_func(q,dq,tau);

% ddq_aug = M_aug_func(q)^-1*f_aug_func(q,dq,tau);

[ddq_aug, lsout.flag, lsout.relres, lsout.iter, lsout.resvec] ...
    = lsqr(M_aug_out, f_aug_out, 1e-6, 40);
% disp( [ 't = ' num2str(t) ' s.  flag = '  num2str(lsout.flag) ...
%     '.  iter = ' num2str(lsout.iter) '.  relres = ' num2str(lsout.relres) ...
%     '.  res = ' num2str(lsout.resvec(end)) '.'] )


ddq = ddq_aug(1:numel(y)/2);

% ddq = M_func(q)^-1*f_func(q,dq,tau);

% bl = -1e8; % lower bound
% bu = 1e8; % upper bound
% ddq=min(max(ddq,bl),bu);


dy = zeros(size(y)); % Initialize derivative vector
dy(1:2:end) = dq;
dy(2:2:end) = ddq;