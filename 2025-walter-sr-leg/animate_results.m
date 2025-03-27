% Animate the results.
% Author: Christian Hubicki. Modified by Adwait Mane in 2025 March.

close all

load("data.mat")

v = VideoWriter('v2-test.mp4','MPEG-4');
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
% axis([-6 15 -1 4])
axis([-6 15 -5 5])

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

% alpha_terr = [p; -0.4142] ; % Horizontal line.
terr_plotVal = ones(numel(p_plotVal),1) * -0.75;
plot_terr = plot(p_plotVal, terr_plotVal, 'k-','linewidth',1) ;

% % plot_terr = plot(p_plot, sin(p_plot), 'k-') ;
% terr_plotVal = terrain_hill(p_plotVal) ;
% % plot_terr = fill([terr_plotVal(1,:), terr_plotVal(1,end), terr_plotVal(1,1)], [terr_plotVal(2,:), -2 -2], 'k', 'facecolor', [1 1 1]*0.75) ;
% plot_terr = plot([terr_plotVal(1,:), terr_plotVal(1,end), terr_plotVal(1,1)], [terr_plotVal(2,:), -2 -2], 'k-','linewidth',1) ;

plot_terr.HandleVisibility = "off" ;

set(gcf,'color',[1 1 1])



for iter = 1:numel(t_anim)

%     subplot(2,2,[3 4])

    j_coords = rj_func(q_anim(iter,:).');
    c_coords = rc_func(q_anim(iter,:).');
    CoM_coords = CoM_func(q_anim(iter,:).');
    % Rearrangement for easy drawing
    % plot_coords = j_coords(:,[9,8,7,6,2,3,4,5]);
    plot_coords = j_coords(:,[1,2,3,4,5]);

    cla
    hold on


    fw_rotA = sum(q_anim(iter,[3,4,6]));
    fw_rotB = sum(q_anim(iter,[3,4,5]));

    % bw_rotA = sum(q_anim(iter,[3,8,9,10]));
    % bw_rotB = sum(q_anim(iter,[3,8,9,11]));






    % plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)

%     plot(c_coords(1,:),c_coords(2,:),'rx')
%     plot(CoM_coords(1,:), CoM_coords(2,:), 'ro')

    % Plot rounded rectangle
    % plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)
    R = 0.2;
    R2 = 0.4;
    color_tan = [201 177 141]/255;
    color_dark_grey = [0.2 0.2 0.2];
    color_light_grey = [191 201 202]/255;
    color_yellow = [247 220 111]/255;
    R_pin = 0.08;
    rim_ratio = 0.9;
    R_com = 0.23;

    % MAIN BODY
%     prect = RoundRectangle(plot_coords(:,4), plot_coords(:,5), R2, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', color_tan)
% 
%     prect = RoundRectangle(plot_coords(:,3), plot_coords(:,4), R, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', color_tan)
% 
% 
% 
% 
%     prect = RoundRectangle(plot_coords(:,5), plot_coords(:,6), R, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', color_tan)
% 
% 
%     prect = RoundRectangle(plot_coords(:,1), plot_coords(:,2), R, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', color_tan)
% %     prect = RoundRectangle(plot_coords(:,5), plot_coords(:,5), R_pin, 0);
% %     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
% 
%     prect = RoundRectangle(plot_coords(:,7), plot_coords(:,8), R, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', color_tan)
% 
%     fill(plot_coords(1,1)+params.L9*wheel_points(1,:),plot_coords(2,1)+params.L9*wheel_points(2,:),'c','facecolor',color2)
%     fill(plot_coords(1,1)+params.L9*rim_ratio*wheel_points(1,:),plot_coords(2,1)+params.L9*rim_ratio*wheel_points(2,:),'c','facecolor',color_tan)
%     plot(plot_coords(1,1)+[0 params.L9*cos(bw_rotB)],plot_coords(2,1)+[0 params.L9*sin(bw_rotB)],'k-')
% 
%     fill(plot_coords(1,2)+params.L8*wheel_points(1,:),plot_coords(2,2)+params.L8*wheel_points(2,:),'c','facecolor',color2)
%     fill(plot_coords(1,2)+params.L8*rim_ratio*wheel_points(1,:),plot_coords(2,2)+params.L8*rim_ratio*wheel_points(2,:),'c','facecolor',color_tan)
%     plot(plot_coords(1,2)+[0 params.L8*cos(bw_rotA)],plot_coords(2,2)+[0 params.L8*sin(bw_rotA)],'k-')
% 
    fill(plot_coords(1,end)+params.L4*wheel_points(1,:), ...
        plot_coords(2,end)+params.L4*wheel_points(2,:), ...
        'g','facecolor',color_dark_grey) % Outer disc. Rim after overlay.
    fill(plot_coords(1,end)+params.L4*rim_ratio*wheel_points(1,:), ... 
        plot_coords(2,end)+params.L4*rim_ratio*wheel_points(2,:), ... 
        'g','facecolor',color_tan) % Overlaid inner disc. Wheel after overlay.
    plot(plot_coords(1,end)+[0 params.L4*cos(fw_rotA)], ... 
        plot_coords(2,end)+[0 params.L4*sin(fw_rotA)],'k-') % Spoke.
% 
    fill(plot_coords(1,end-1)+params.L5*wheel_points(1,:),plot_coords(2,end-1)+params.L5*wheel_points(2,:),'g','facecolor',color_dark_grey)
    fill(plot_coords(1,end-1)+params.L5*rim_ratio*wheel_points(1,:),plot_coords(2,end-1)+params.L5*rim_ratio*wheel_points(2,:),'g','facecolor',color_light_grey)
    plot(plot_coords(1,end-1)+[0 params.L5*cos(fw_rotB)],plot_coords(2,end-1)+[0 params.L5*sin(fw_rotB)],'k-')
% 
% 
%     prect = RoundRectangle(plot_coords(:,1), plot_coords(:,1), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,2), plot_coords(:,2), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,3), plot_coords(:,3), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,4), plot_coords(:,4), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,5), plot_coords(:,5), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,6), plot_coords(:,6), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,7), plot_coords(:,7), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])
%     prect = RoundRectangle(plot_coords(:,8), plot_coords(:,8), R_pin, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [0 0 0])

% PLOT COM SYMBOL
%     prect = RoundRectangle([c_coords(1,1);c_coords(2,1)], [c_coords(1,1);c_coords(2,1)], R_com, 0);
%     fill(prect(1,:), prect(2,:), 'k', 'facecolor', [1 1 1])
%     com_ul = [R_com*cos(linspace(pi/2,pi,100)) 0 0; R_com*sin(linspace(pi/2,pi,100)) 0 R_com];
%     com_ul = com_ul + [c_coords(1,1); c_coords(2,1)]*[0*com_ul(1,:)+1];
%     fill(com_ul(1,:), com_ul(2,:), 'k', 'facecolor', [0 0 0],'edgecolor','none')
%     com_lr = [R_com*cos(linspace(3*pi/2,2*pi,100)) 0 0; R_com*sin(linspace(3*pi/2,2*pi,100)) 0 R_com];
%     com_lr = com_lr + [c_coords(1,1); c_coords(2,1)]*[0*com_lr(1,:)+1];
%     fill(com_lr(1,:), com_lr(2,:), 'k', 'facecolor', [0 0 0],'edgecolor','none')


    plot(plot_coords(1,:), plot_coords(2,:), 'k-', 'linewidth', 4)


    axis off
    hold off
    drawnow

    %     pause

    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);
