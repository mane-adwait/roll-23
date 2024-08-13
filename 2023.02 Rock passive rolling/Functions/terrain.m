
function out = terrain(p)

par = getParameters_v0();

if par.terrain == "quartic"
    scale_x = 1/5;
    x = p;
    z = terrain_quartic(scale_x*p) ;

elseif par.terrain == "parabolic"
    x = p;
%     z = -p.^2 ;
    z = -(1/10)*p.^2 ;

else
    disp('Error: the selected terrain is not supported.')
end



%% Quartic.

% scale_x = 1/5;
% x = p;
% z = terrain_quartic(scale_x*p) ;

%% Cubic.


%% Parabolic.

% x = p;
% z = -p.^2 ;

%% Circular.

% par.r2 = 1; % Circular terrain.


%% Linear.
% alpha_parab = [p; tan(deg2rad(-5))*p] ; 

% z = tan(deg2rad(-5))*p ;

%% Create the output vector.
out = [x; z] ;

end
