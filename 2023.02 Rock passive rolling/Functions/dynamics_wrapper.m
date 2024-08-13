
function dstate = dynamics_wrapper(t, state)

[M_aug_inv, d2q_aug, d2q, lambda, dstate, lsout] = dynamics(t, state) ;

end