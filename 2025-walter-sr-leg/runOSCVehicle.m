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

% 2025 May:
t4_0 = 0; % Assume for all configurations.
% User-defined: p_0, q4_0, q5_0, q6_0.
p_0 = 0; q4_0 = pi/2; q5_0 = pi/4; q6_0 = 0; % Crouched configuration.
% p_0 = 0; q4_0 = pi/2; q5_0 = pi/16; q6_0 = 0; % Config-A.
% p_0 = 0; q4_0 = -pi/2; q5_0 = 3*pi/4; q6_0 = 0; % Forward knee configuration.


% Derived: q1_0, q2_0, q3_0, phi_0.

q3_0 = t4_0 - q4_0 - q5_0;

% Derive q1_0, q2_0:
alpha_terr_0 = alpha_terr_func(p_0);
% Find the exterior unit normal to the terrain.
% 'Exterior' means it points to the exterior of the terrain at all points.
% In constrast, the usual unit normal points towards the concave side.
N_ute_0 = RM_CCW(pi/2) * T_terr_func(p_0) ;
p2fwA_0 = params.wheel_radius * N_ute_0 ; % Scaled by the wheel radius.
base_0 = alpha_terr_0 + p2fwA_0 ...
    - params.L3a*[cos(pi-(q3_0+q4_0)); sin(pi-(q3_0+q4_0))] ...
    - params.L2*[cos(pi-q3_0); sin(pi-q3_0)] + [0; params.L1a] ;
q1_0 = base_0(1); q2_0 = base_0(2);

% Derive phi_0:
T_terr_0 = T_terr_func(p_0);
A_angle = atan2(T_terr_0(2),T_terr_0(1)); % Angle w.r.t. +X axis.
phi_0 = pi/2 - A_angle;

q0 = [q1_0; q2_0; q3_0; q4_0; q5_0; q6_0; phi_0; p_0] ;
% -----------------------------------------------------------

% % 2025 March:
% % User-defined: p_0, q4_0, q3_0.
% p_0 = 0; q3_0 = -3*pi/4; q4_0 = pi/2;
% % Hard-coded: wheel angles q5_0 and q6_0.
% q5_0 = -(q3_0 + q4_0); q6_0 = 0; 
% % Derived: phi_0, q1_0, q2_0.
% phi_0 = 0;
% % q1_0 = NaN;
% % q2_0 = NaN;
% 
% % -----------------------------------------------------------
% % Added 2023 Sep 16.
% 
% % Initial state
% 
% % p_0 = -pi/2 - pi/4;
% p_0 = 3.5652 ; % From fsolve.
% 
% alpha_terr_0 = alpha_terr_func(p_0);
% 
% % Find the exterior unit normal to the terrain.
% % 'Exterior' means it points to the exterior of the terrain at all points.
% % In constrast, the usual unit normal points towards the concave side.
% N_ute_0 = RM_CCW(pi/2) * T_terr_func(p_0) ;
% p2fwA_0 = params.wheel_radius * N_ute_0 ; % Scaled by the wheel radius.
% 
% A_angle = atan2(N_ute_0(2), N_ute_0(1)) ; % Angle w.r.t. +X axis.
% B_angle = pi/2 - A_angle ; % Angle w.r.t. -Y axis.
% phi_0 = -pi/2 - B_angle ;
% % phi_0 = -pi/2 ;
% % -----------------------------------------------------------
% 
% angle_shin = pi-(q3_0+q4_0);
% base0 = alpha_terr_0 + p2fwA_0 + ...
%     params.L3a*[cos(angle_shin); sin(angle_shin)] + ...
%     params.L2*[cos(pi-q3_0); sin(pi-q3_0)] - ...
%     [params.L1b; 0];
% 
% q0 = [base0(1); base0(2); q3_0; q4_0; q5_0; q6_0; phi_0; p_0] ;
% -----------------------------------------------------------

% q0_phys = zeros(6,1);
% % q0_phys(2) = 0.75;
% q0 = [q0_phys; phi_0; p_0] ;
% -----------------------------------------------------------

% q0 = [1 1 0 -pi/4 pi/4 0 0 -3*pi/4 3*pi/4 0 0].';
% dq0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0].';
% dq0 = [0 0 0 0 0 0 0 0].';
dq0 = zeros(8,1);

% Rearrange into q1; dq1; q2; dq2 ... ordering
y0 = reshape([q0.';dq0.'],[numel(q0)*2,1]);

sim_time = 5; %7.5; %5; % Simulation run time.
Ts = 0.01; % Controller time-step.

% Test OSC.
height_des = 3.0 ;
xdes = @(t) [0; height_des]; % constant height


% % Simple Drive Trajectory
% height_des = 2.1 ;
% % xdes = @(t) [0; height_des; 0]; % constant height
% % xdes = @(t) [2*t; height_des; 0]; % constant height
% xdes = @(t) [2*t; height_des; 0; 0; 0; 0; 0]; % constant height
% % xdes = @(t) [2*t; height_des; 0; height_des; height_des]; % constant height
% % xdes = @(t) [2*t; 3; 0]; % constant height
% % xdes = @(t) [2*t; 2; 0; 3; 3; 0.75];
% % Jiggle Drive Trajectory
% % xdes = @(t) [2*t; 1.1-0.1*sin(6*t); 0.15*sin(6*t)]; % oscillating height

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
    
    % Extract generalized coordinates (q) from state vector
    q = y0(1:2:end);
    % Extract dq from state vector
    dq = y0(2:2:end);
    
    % Compute torques (once per Ts)
    tau = GetTorqueOSC(xdes(t_start),dxdes(t_start),q,dq);

    disp(['t_start = ' num2str(t_start) '    tau = ' ...
        num2str(tau(1)) '  ' num2str(tau(2)) '  ' num2str(tau(3)) ])

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

save("v8-1-data.mat")
