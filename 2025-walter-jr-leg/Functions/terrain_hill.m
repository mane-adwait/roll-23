
function out = terrain_hill(p)

params = getVehicleParams();

mu = 7.5;
sigma = 1;
A = 1.5;
x = p ;
y = A*exp(-0.5*((p-mu)/sigma).^2);


%% Create the output vector.
out = [x; y] ;

end
