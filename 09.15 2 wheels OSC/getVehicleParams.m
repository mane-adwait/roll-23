% getVehicleParams

function params = getVehicleParams()

% Gravity
params.g = 9.81;

% Wheel and terrain geometry.
params.wheel_radius = 0.75 ; 
params.terrain_radius = 5 ; 

% Main Body (1)
params.m1 = 1; % Mass of body 1
params.L1a = 1.0;  % Length to front joint
params.L1b = 1.0;  % length to back joint
params.I1 = 1; % inertia

% Front Thigh (2)
params.m2 = 1; % Mass of body 1
params.L2 = 2;  % Length to front wheel
params.I2 = 1; % inertia

% Front Shin (3)
params.m3 = 1; % Mass of body 1
params.L3a = 1;  % Length to front joint
params.L3b = 1;  % length to back joint
params.I3 = 1/3; % inertia

% Front Wheel A (4)
params.m4 = 1; % Mass of body 1
params.L4 = 0.75;  % Length to front wheel
params.I4 = 1/3; % inertia

% Front Wheel B (5)
params.m5 = 1; % Mass of body 1
params.L5 = 0.75;  % Length to front wheel
params.I5 = 1/3; % inertia


% Back Thigh (6)
params.m6 = 1; % Mass of body 1
params.L6 = 2;  % Length to front wheel
params.I6 = 1/3; % inertia

% Back Shin (7)
params.m7 = 1; % Mass of body 1
params.L7a = 1;  % Length to front joint
params.L7b = 1;  % length to back joint
params.I7 = 1/3; % inertia

% Back Wheel A (8)
params.m8 = 1; % Mass of body 1
params.L8 = 0.75;  % Length to front wheel
params.I8 = 1/3; % inertia

% Back Wheel B (9)
params.m9 = 1; % Mass of body 1
params.L9 = 0.75;  % Length to front wheel
params.I9 = 1/3; % inertia