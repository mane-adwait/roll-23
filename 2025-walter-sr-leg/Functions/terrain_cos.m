
function out = terrain(p)

params = getVehicleParams();

x = p ;
y = 0.5 * cos(p) ;


%% Create the output vector.
out = [x; y] ;

end
