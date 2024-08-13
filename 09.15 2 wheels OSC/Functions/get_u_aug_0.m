function u_aug = get_u_aug_0(t, nq_aug)

u = 0;
% u = 12; % N*m.
% u = 0.25*sin(t/2);
% u = 0.5*sin(t);
% u = 0.25*sin(2*t);

u_aug = zeros(nq_aug,1); u_aug(3) = u;

end
