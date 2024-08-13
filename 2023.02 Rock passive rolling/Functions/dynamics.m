function [M_aug_inv, d2q_aug, d2q, lambda, dstate, lsout] = dynamics(t, state)

nq = numel(state)/2;
q = state(1:nq); dq = state(nq+1:end);

M_aug = get_M_aug(state);
% M = M_aug(1:5,1:5); % Inertia matrix.
% A = M_aug(6:end, 1:5); % Constraint Jacobian.
f_aug = get_f_aug(state);

u_aug = get_u_aug_0(t, numel(f_aug) );

% M_aug_inv = M_aug^-1;
% M_aug_pinv = pinv(M_aug);
M_aug_inv = NaN; % If we need to retrive M_aug_inv for debugging.

% d2q_aug = M_aug_inv * (f_aug + u_aug);
[d2q_aug, lsout.flag, lsout.relres, lsout.iter, lsout.resvec] = lsqr(M_aug, f_aug + u_aug, 1e-6, 40);
disp( [ 't = ' num2str(t) ' s.  flag = '  num2str(lsout.flag) ...
    '.  iter = ' num2str(lsout.iter) '.  relres = ' num2str(lsout.relres) ...
    '.  res = ' num2str(lsout.resvec(end)) '.'] )
% rank(M_aug) % display for debugging.

d2q = d2q_aug(1:nq);
lambda = d2q_aug(nq+1:end);

dstate = [dq; d2q];
% t % for debugging.

end
