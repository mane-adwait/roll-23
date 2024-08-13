
function dstate = dynamics_wrapper(t, state, u)

[M_aug_inv, d2q_aug, d2q, lambda, dstate, lsout] = dynamics(t, state, u) ;

end