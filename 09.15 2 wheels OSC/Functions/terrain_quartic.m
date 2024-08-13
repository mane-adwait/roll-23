
function z = terrain_quartic(p)

% Quartic parameters. Migrate to getParameters.m
% a4 = 1/4; a3 = 0; a2 = -3/4; a1 = 0; a0 = 0;
% a4 = 2; a3 = 0; a2 = -1/4; a1 = 0; a0 = 0;
a4 = 3; a3 = 0; a2 = -5; a1 = 0; a0 = 0;

z = a4*p.^4 + a3*p.^3 + a2*p.^2 + a1*p + a0;

end