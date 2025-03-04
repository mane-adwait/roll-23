
% Exercise 7.36 3rd Edition
% q1 = phi

syms q1 dq1 Omega k L eps m g ;
assume(q1,'real'); assume(dq1,'real'); assume(Omega,'real'); 
assume(k,'real'); assume(L,'real'); assume(eps,'real'); assume(m,'real');
assume(g,'real');
% assumptions

IG = (m*L^2)/12;
rBA = eps*[cos(q1); -sin(q1); 0];
rGB = L*[1; 0; 0]/2;
K = [0; 0; 1];
omega = (dq1-Omega)*K;
vG = cross(-Omega*K,rBA) + cross(omega,rGB);
T = (m*vG.'*vG)/2 + (IG*(dq1-Omega)^2)/2; % Note: a.'*b = transpose(a)*b
V = (k*q1^2)/2;
Q1 = 0;

% All time varying variables go where {q1} is. Note that if 'q1' is
% specified as an argument for fulldiff, then dq1 is assumed to be time
% dependent as well.

eq_q1 = fulldiff(diff(T,dq1),{q1}) - diff(T,q1) + diff(V,q1) - Q1;
% lprint(eq_q1); % Display equation of motion in LaTeX
eq_q1 = simplify(eq_q1);
disp('eq_q1:'); pretty(eq_q1); % Display equation of motion in command window



% ans =
%
% (d2q1*m*L^2)/3 + (eps*m*sin(q1)*L*Omega^2)/2 + k*q1