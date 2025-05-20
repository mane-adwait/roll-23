% getVehicleParams

function params = getVehicleParams()

% SI units. Meters, kilograms, seconds.

% Gravity
params.g = 9.81;

% ---------- Walter Sr. values ----------

% Wheel and terrain geometry.
params.wheel_radius = 0.064 ; 
params.terrain_radius = 5 ; 

% Main Body (1)
params.m1 = 2.5; % Mass of body 1
params.L1a = 0.02;  % Length to front joint
params.I1 = (1/12)*params.m1*params.L1a^2; % inertia

% Front Thigh (2)
params.m2 = 0.4; % Mass of body 2
params.L2 = 0.102;  % Length to front wheel
params.I2 = (1/12)*params.m2*params.L2^2; % inertia

% Front Shin (3)
params.m3 = 1.3; % Mass of body 1
params.L3a = 0.159/2;  % Length to front joint
params.L3b = 0.159/2;  % length to back joint
params.I3 = (1/12)*params.m3*(params.L3a+params.L3b)^2; % inertia

% Front Wheel A (4)
params.m4 = 0.4; % Mass of body 1
params.L4 = params.wheel_radius;  % Length to front wheel
params.I4 = (1/2)*params.m4*params.L4^2; % inertia

% Front Wheel B (5)
params.m5 = 0.4; % Mass of body 1
params.L5 = params.wheel_radius;  % Length to front wheel
params.I5 = (1/2)*params.m5*params.L5^2; % inertia

% ---------- Arbitrary values ----------

% % Wheel and terrain geometry.
% params.wheel_radius = 0.75 ; 
% params.terrain_radius = 5 ; 
% 
% % Main Body (1)
% params.m1 = 1; % Mass of body 1
% params.L1a = 1.0;  % Length to front joint
% params.L1b = 1.0;  % length to back joint
% params.I1 = 1; % inertia
% 
% % Front Thigh (2)
% params.m2 = 1; % Mass of body 1
% params.L2 = 2;  % Length to front wheel
% params.I2 = 1; % inertia
% 
% % Front Shin (3)
% params.m3 = 1; % Mass of body 1
% params.L3a = 1;  % Length to front joint
% params.L3b = 1;  % length to back joint
% params.I3 = 1/3; % inertia
% 
% % Front Wheel A (4)
% params.m4 = 1; % Mass of body 1
% params.L4 = params.wheel_radius;  % Length to front wheel
% params.I4 = 1/3; % inertia
% 
% % Front Wheel B (5)
% params.m5 = 1; % Mass of body 1
% params.L5 = params.wheel_radius;  % Length to front wheel
% params.I5 = 1/3; % inertia