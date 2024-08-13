% See reamde_v*.txt for full instructions.

function par = getParameters_v0()
% SI units

% Terrain geometry options: 
% "linear", "circular", "parabolic", "cubic", "quartic"
par.terrain = "parabolic" ;

if par.terrain == "quartic"
    par.p_start = -6.7 ;
    par.t_end = 12 ;

elseif par.terrain == "parabolic"
    par.p_start = 0.1 ;
%     par.t_end = 5 ;
    par.t_end = 5 ;
else
    disp('Error: the selected terrain is not supported.')

end

%% Wheel parameters.

% Geometry
par.r1 = 0.25; 

% Mass and rotational inertia
par.m = 1; par.I = par.m * par.r1^2;

%% Other.

par.g = 9.81;

par.theta_start = 0; % Aug 01. Assumed to be zero elsewhere in the code.



end