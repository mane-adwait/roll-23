% runOSCVehicle.m

clc
clear
close all
addpath(genpath( 'Functions' ) );
addpath(genpath( 'auto' ) );


warning('off','MATLAB:nearlySingularMatrix')

tic

% Retrieve parameters from external function
params = getVehicleParams();

% -----------------------------------------------------------
% Added 2023 Sep 16.

% Initial state

% fwA to base for standard initial configuration:
fwA2base = [-3.41421 ; 1.41421] ;

% p0 = -pi/2 - pi/4;
p0 = 3.5652 ; % From fsolve.

alpha_terr_0 = alpha_terr_func(p0);

% Find the exterior unit normal to the terrain.
% 'Exterior' means it points to the exterior of the terrain at all points.
% In constrast, the usual unit normal points towards the concave side.
N_ute_0 = RM_CCW(pi/2) * T_terr_func(p0) ;
p2fwA_0 = params.wheel_radius * N_ute_0 ; % Scaled by the wheel radius.

p2base = p2fwA_0 + fwA2base ;
base0 = alpha_terr_0 + p2base ;

A_angle = atan2(N_ute_0(2), N_ute_0(1)) ; % Angle w.r.t. +X axis.
B_angle = pi/2 - A_angle ; % Angle w.r.t. -Y axis.
phi0 = -pi/2 - B_angle ;
% phi0 = -pi/2 ;

q0_phys = zeros(6,1);
% q0_phys(2) = 0.75;
q0 = [q0_phys; phi0; p0] ;


% -----------------------------------------------------------

% q0 = [1 1 0 -pi/4 pi/4 0 0 -3*pi/4 3*pi/4 0 0].';
% dq0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0].';
% dq0 = [0 0 0 0 0 0 0 0].';
dq0 = zeros(8,1);

% Rearrange into q1; dq1; q2; dq2 ... ordering
y0 = reshape([q0.';dq0.'],[numel(q0)*2,1]);

sim_time = 3; %7.5; %5; % Simulation run time
Ts = 0.01; % Sample time (for controller)

% Simple Drive Trajectory
height_des = 2.1 ;
% xdes = @(t) [0; height_des; 0]; % constant height
% xdes = @(t) [2*t; height_des; 0]; % constant height
xdes = @(t) [2*t; height_des; 0; 0; 0; 0; 0]; % constant height
% xdes = @(t) [2*t; height_des; 0; height_des; height_des]; % constant height
% xdes = @(t) [2*t; 3; 0]; % constant height
% xdes = @(t) [2*t; 2; 0; 3; 3; 0.75];
% Jiggle Drive Trajectory
% xdes = @(t) [2*t; 1.1-0.1*sin(6*t); 0.15*sin(6*t)]; % oscillating height

% Integrator options
% options = odeset('OutputFcn',@odeplot);

% Differentiate position trajectory for desired velocities
syms t
dxdes = diff(xdes(t),t);
dxdes = matlabFunction(dxdes,'vars',t);

% Initialize system variable trajectories to empty
t_out = [];
y_out = [];
u_out = [];

for t_start = 0:Ts:(sim_time-Ts)
    
    t_start
    
    % Extract generalized coordinates (q) from state vector
    q = y0(1:2:end);
    % Extract dq from state vector
    dq = y0(2:2:end);
    
    % Compute torques (once per Ts)
    % tau = GetTorqueOSC(xdes(t_start),dxdes(t_start),q,dq);
    tau = zeros(2,1);
    
    % Define inline dynamics function for passing constant torque
    dyn = @(t,y) dynVehicleControl(t,y,tau);
    % Run numerical simulation (nonlinear integrator)
    % for the duration of Ts
    %     [t_sim, y_sim] = ode45(dyn, [t_start t_start+Ts], y0, options);
    [t_sim, y_sim] = ode45(dyn, [t_start t_start+Ts], y0);
    
    % Store computed torques for future plotting
    [~,u_sim] = meshgrid(t_sim,tau);
    u_sim = u_sim.';
    
    % Append data from system variables
    t_out = [t_out; t_sim(2:end)];
    y_out = [y_out; y_sim(2:end,:)];
    u_out = [u_out; u_sim(2:end,:)];
    
    % Update initial condition for next sample time, Ts
    y0 = y_sim(end,:).';
end

toc


%% Animation setup.
FPS = 20;
SLOMO = 1;

t_anim = (min(t_out):1/FPS/SLOMO:max(t_out)).';
y_anim = interp1(t_out,y_out,t_anim);

q_anim = y_anim(:,1:2:end);

save("v2-data.mat")
