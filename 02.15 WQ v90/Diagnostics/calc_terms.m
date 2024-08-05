% Friday 2022 September 23

function [term1, term2, A_sigma1] = calc_terms(state)

% Unpack
theta = state(3,:) ;
phi = state(4,:) ;
p = state(5,:) ;

size_state = size(state) ;
nk = size_state(2) ; % No. of timesteps.

term1 = NaN(1,nk) ;
term2 = NaN(1,nk) ;
A_sigma1 = NaN(1,nk) ;

for k = 1:nk

term1(1,k) = cos(phi(k) + theta(k)) ;
term2(1,k) = 2*p(k)*sin(phi(k) + theta(k)) ;
    
% sigma1 = cos(phi + theta) + 2*p*sin(phi + theta)
A_sigma1(1,k) = cos(phi(k) + theta(k)) + 2*p(k)*sin(phi(k) + theta(k)) ;

end

end