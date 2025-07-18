% testDynamics.m

clc
clear
close all
addpath(genpath( 'Functions' ) );
addpath(genpath( 'auto' ) );

params = getVehicleParams();

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

q0_phys = [base0(1) base0(2) 0 -pi/4 pi/4 0 0 -3*pi/4 3*pi/4 0 0].' ; % 11 values.
% q0 = [q0_phys; phi0; p0] ;
q0 = [q0_phys; phi0; p0; phi_bwB_0; p_bwB_0] ;


% dq0 = [0 0 0 0 0 0 0 0 0 0 0 0 0].';
dq0 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0].';

% Initial state
% Rearrange into q1; dq1; q2; dq2 ... ordering
y0 = reshape([q0.';dq0.'],[numel(q0)*2,1]);


%%

sim_time = 4;

options = odeset('OutputFcn',@odeplot);
% options = odeset('AbsTol',1e-6,'RelTol',1e-3, ... % default tolerances.
%     'Stats','on', 'OutputFcn',@odeplot);


[t_out, y_out] = ode45(@dynamicsVehicle, [0 sim_time], y0, options);

FPS = 30;
SLOMO = 2;

t_anim = (min(t_out):1/FPS/SLOMO:max(t_out)).';
y_anim = interp1(t_out,y_out,t_anim);

q_anim = y_anim(:,1:2:end);

videoObj = VideoWriter('v0 testDynamics.mp4','MPEG-4');
videoObj.Quality = 95;
open(videoObj);

figure(2)

% Set the figure position using the syntax [left bottom width height].
% set(gcf,'outerposition',[496 388 1185 638])
set(gcf,'Units','normalized'); 
set(gcf,'OuterPosition',[0.5 0.05 0.5 0.7]) % Bottom right quadrant of screen.

axis equal
grid on
% axis([-2.5 12.5 -2.5 2.5]) % [xmin xmax ymin ymax]
axis([-10 10 -7 4])
% axis([-10 10 -10 8]) % 09.13 Circular terrain.

NUM_WHEEL_POINTS = 200;
wheel_points = [cos(linspace(0,2*pi,NUM_WHEEL_POINTS));sin(linspace(0,2*pi,NUM_WHEEL_POINTS))];

hold on

%% Plot the terrain.

% Circular terrain.
% p_plot = -pi/8:0.1:2*pi ;
% plot_terr = plot(params.terrain_radius * cos(p_plot), params.terrain_radius * sin(p_plot), 'k-') ;

p_plotVal = -10:0.1:20 ;
% plot_terr = plot(p_plot, sin(p_plot), 'k-') ;
terr_plotVal = terrain_cos(p_plotVal) ;
plot_terr = plot(terr_plotVal(1,:), terr_plotVal(2,:), 'k-') ;

plot_terr.HandleVisibility = "off" ;
%%

% Store the values of the center of fwA in an array.
for iter = 1:numel(t_anim)
    c_coords = rc_func(q_anim(iter,:).') ;
    % fwA_center: column corresponds to time.
    fwA_center(:,iter) = c_coords(:,4) ;
end

% Plot the center of fwA.
plot_fwA_center = plot(fwA_center(1,:), fwA_center(2,:), 'b--') ;
plot_fwA_center.HandleVisibility = "off" ;


for iter = 1:numel(t_anim)

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


    fill(plot_coords(1,1)+params.L9*wheel_points(1,:),plot_coords(2,1)+params.L9*wheel_points(2,:),'c')
    plot(plot_coords(1,1)+[0 params.L9*cos(bw_rotB)],plot_coords(2,1)+[0 params.L9*sin(bw_rotB)],'k-')

    fill(plot_coords(1,2)+params.L8*wheel_points(1,:),plot_coords(2,2)+params.L8*wheel_points(2,:),'c')
    plot(plot_coords(1,2)+[0 params.L8*cos(bw_rotA)],plot_coords(2,2)+[0 params.L8*sin(bw_rotA)],'k-')

    fill(plot_coords(1,end-1)+params.L5*wheel_points(1,:),plot_coords(2,end-1)+params.L5*wheel_points(2,:),'g')
    plot(plot_coords(1,end-1)+[0 params.L5*cos(fw_rotB)],plot_coords(2,end-1)+[0 params.L5*sin(fw_rotB)],'k-')

    fill(plot_coords(1,end)+params.L4*wheel_points(1,:),plot_coords(2,end)+params.L4*wheel_points(2,:),'g')
    plot(plot_coords(1,end)+[0 params.L4*cos(fw_rotA)],plot_coords(2,end)+[0 params.L4*sin(fw_rotA)],'k-')



    plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)
    
    plot(c_coords(1,:),c_coords(2,:),'rx')
    plot(CoM_coords(1,:), CoM_coords(2,:), 'ro')
    

    hold off
    drawnow

    frame = getframe(gcf);
    writeVideo(videoObj,frame);
end

close(videoObj)