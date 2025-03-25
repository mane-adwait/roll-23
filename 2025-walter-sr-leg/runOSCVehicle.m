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
% Added Sep 16.

% Initial state

% fwA to base for standard initial configuration:
fwA2base = [-3.41421 ; 1.41421] ;

% p0 = -pi/2 - pi/4;
p0 = 3.5652 ; % From fsolve.

alpha_terr_0 = alpha_terr_func(p0);

% Find the exterior unit normal to the terrain.
% 'External' means it points to the exterior of the terrain at all points.
% In constrast, the usual unit normal points towards the concave side.
N_ute_0 = RM_CCW(pi/2) * T_terr_func(p0) ;
p2fwA_0 = params.wheel_radius * N_ute_0 ; % Scaled by the wheel radius.

p2base = p2fwA_0 + fwA2base ;
base0 = alpha_terr_0 + p2base ;

A_angle = atan2(N_ute_0(2), N_ute_0(1)) ; % Angle w.r.t. +X axis.
B_angle = pi/2 - A_angle ; % Angle w.r.t. -Y axis.
phi0 = -pi/2 - B_angle ;
% phi0 = -pi/2 ;

phi_bwB_0 = -pi/2 + B_angle ; p_bwB_0 = -p0 ;

% q0_phys = [base0(1) base0(2) 0 -pi/4 pi/4 0 0 -3*pi/4 3*pi/4 0 0].' ; % 11 values.
% % q0 = [q0_phys; phi0; p0] ;
% q0 = [q0_phys; phi0; p0; phi_bwB_0; p_bwB_0] ;

q0_phys = zeros(6,1);
% q0 = [q0_phys; phi0; p0] ;
q0 = [q0_phys; phi0; p0] ;


% -----------------------------------------------------------

% q0 = [1 1 0 -pi/4 pi/4 0 0 -3*pi/4 3*pi/4 0 0].';
% dq0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0].';
% dq0 = [0 0 0 0 0 0 0 0].';
dq0 = zeros(8,1);

% Rearrange into q1; dq1; q2; dq2 ... ordering
y0 = reshape([q0.';dq0.'],[numel(q0)*2,1]);

sim_time = 1; %7.5; %5; % Simulation run time
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


%% Animation
FPS = 20;
SLOMO = 1;

t_anim = (min(t_out):1/FPS/SLOMO:max(t_out)).';
y_anim = interp1(t_out,y_out,t_anim);

q_anim = y_anim(:,1:2:end);

save("data.mat")
return

%%

v = VideoWriter('v2024-OSC-testCH-1.mp4','MPEG-4');
v.Quality = 99;
v.FrameRate = FPS;
open(v);

figure(2)

% Set the figure position using the syntax [left bottom width height].
% set(gcf,'outerposition',[496 388 1185 638])
set(gcf,'Units','normalized');
set(gcf,'OuterPosition',[0.5 0.05 0.5 0.7]) % Bottom right quadrant of screen.

axis equal
grid off
% [xmin xmax ymin ymax]
axis([-6 15 -1 4])
% axis([-2.5 22.5 -2.5 3.5])

NUM_WHEEL_POINTS = 200;
wheel_points = [cos(linspace(0,2*pi,NUM_WHEEL_POINTS));sin(linspace(0,2*pi,NUM_WHEEL_POINTS))];

hold on

% plot_hdes = plot([-7 15],[height_des height_des], '--', 'color', [0 0 0])
% plot_hdes.HandleVisibility = "off" ;

%% Plot the terrain.

% Circular terrain.
% p_plot = -pi/8:0.1:2*pi ;
% plot_terr = plot(params.terrain_radius * cos(p_plot), params.terrain_radius * sin(p_plot), 'k-') ;

p_plotVal = -10:0.1:20 ;
% plot_terr = plot(p_plot, sin(p_plot), 'k-') ;
terr_plotVal = terrain_hill(p_plotVal) ;
% plot_terr = fill([terr_plotVal(1,:), terr_plotVal(1,end), terr_plotVal(1,1)], [terr_plotVal(2,:), -2 -2], 'k', 'facecolor', [1 1 1]*0.75) ;
plot_terr = plot([terr_plotVal(1,:), terr_plotVal(1,end), terr_plotVal(1,1)], [terr_plotVal(2,:), -2 -2], 'k-','linewidth',1) ;

plot_terr.HandleVisibility = "off" ;

set(gcf,'color',[1 1 1])



for iter = 1:numel(t_anim)

%     subplot(2,2,[3 4])

    j_coords = rj_func(q_anim(iter,:).');
    c_coords = rc_func(q_anim(iter,:).');
    CoM_coords = CoM_func(q_anim(iter,:).');
    % Rearrangement for easy drawing
    plot_coords = j_coords(:,[9,8,7,6,2,3,4,5]);

    cla
    hold on


    fw_rotA = sum(q_anim(iter,[3,4,5,7]));
    fw_rotB = sum(q_anim(iter,[3,4,5,6]));

    bw_rotA = sum(q_anim(iter,[3,8,9,10]));
    bw_rotB = sum(q_anim(iter,[3,8,9,11]));






    plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)

%     plot(c_coords(1,:),c_coords(2,:),'rx')
%     plot(CoM_coords(1,:), CoM_coords(2,:), 'ro')

    % Plot rounded rectangle
    plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)
    R = 0.2;
    R2 = 0.4;
    color1 = [201 177 141]/255;
    color2 = [0.2 0.2 0.2];
    R_pin = 0.08;
    rim_ratio = 0.9;
    R_com = 0.23;

    % MAIN BODY
    prect = RoundRectangle(plot_coords(:,4), plot_coords(:,5), R2, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', color1)

    prect = RoundRectangle(plot_coords(:,3), plot_coords(:,4), R, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', color1)




    prect = RoundRectangle(plot_coords(:,5), plot_coords(:,6), R, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', color1)


    prect = RoundRectangle(plot_coords(:,1), plot_coords(:,2), R, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', color1)
%     prect = RoundRectangle(plot_coords(:,5), plot_coords(:,5), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])

    prect = RoundRectangle(plot_coords(:,7), plot_coords(:,8), R, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', color1)

    fill(plot_coords(1,1)+params.L9*wheel_points(1,:),plot_coords(2,1)+params.L9*wheel_points(2,:),'c','facecolor',color2)
    fill(plot_coords(1,1)+params.L9*rim_ratio*wheel_points(1,:),plot_coords(2,1)+params.L9*rim_ratio*wheel_points(2,:),'c','facecolor',color1)
    plot(plot_coords(1,1)+[0 params.L9*cos(bw_rotB)],plot_coords(2,1)+[0 params.L9*sin(bw_rotB)],'k-')

    fill(plot_coords(1,2)+params.L8*wheel_points(1,:),plot_coords(2,2)+params.L8*wheel_points(2,:),'c','facecolor',color2)
    fill(plot_coords(1,2)+params.L8*rim_ratio*wheel_points(1,:),plot_coords(2,2)+params.L8*rim_ratio*wheel_points(2,:),'c','facecolor',color1)
    plot(plot_coords(1,2)+[0 params.L8*cos(bw_rotA)],plot_coords(2,2)+[0 params.L8*sin(bw_rotA)],'k-')

    fill(plot_coords(1,end-1)+params.L5*wheel_points(1,:),plot_coords(2,end-1)+params.L5*wheel_points(2,:),'g','facecolor',color2)
    fill(plot_coords(1,end-1)+params.L5*rim_ratio*wheel_points(1,:),plot_coords(2,end-1)+params.L5*rim_ratio*wheel_points(2,:),'g','facecolor',color1)
    plot(plot_coords(1,end-1)+[0 params.L5*cos(fw_rotB)],plot_coords(2,end-1)+[0 params.L5*sin(fw_rotB)],'k-')

    fill(plot_coords(1,end)+params.L4*wheel_points(1,:),plot_coords(2,end)+params.L4*wheel_points(2,:),'g','facecolor',color2)
    fill(plot_coords(1,end)+params.L4*rim_ratio*wheel_points(1,:),plot_coords(2,end)+params.L4*rim_ratio*wheel_points(2,:),'g','facecolor',color1)
    plot(plot_coords(1,end)+[0 params.L4*cos(fw_rotA)],plot_coords(2,end)+[0 params.L4*sin(fw_rotA)],'k-')


    prect = RoundRectangle(plot_coords(:,1), plot_coords(:,1), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,2), plot_coords(:,2), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,3), plot_coords(:,3), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,4), plot_coords(:,4), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,5), plot_coords(:,5), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,6), plot_coords(:,6), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,7), plot_coords(:,7), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
    prect = RoundRectangle(plot_coords(:,8), plot_coords(:,8), R_pin, 0);
    fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])

% PLOT COM SYMBOL
%     prect = RoundRectangle([c_coords(1,1);c_coords(2,1)], [c_coords(1,1);c_coords(2,1)], R_com, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [1 1 1])
%     com_ul = [R_com*cos(linspace(pi/2,pi,100)) 0 0; R_com*sin(linspace(pi/2,pi,100)) 0 R_com];
%     com_ul = com_ul + [c_coords(1,1); c_coords(2,1)]*[0*com_ul(1,:)+1];
%     fill(com_ul(1,:), com_ul(2,:), 'k', 'facecolor', [0 0 0],'edgecolor','none')
%     com_lr = [R_com*cos(linspace(3*pi/2,2*pi,100)) 0 0; R_com*sin(linspace(3*pi/2,2*pi,100)) 0 R_com];
%     com_lr = com_lr + [c_coords(1,1); c_coords(2,1)]*[0*com_lr(1,:)+1];
%     fill(com_lr(1,:), com_lr(2,:), 'k', 'facecolor', [0 0 0],'edgecolor','none')


    axis off
    hold off
    drawnow

    %     pause

    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);


