% See reamde_v*.txt for full instructions.

close all; clear all;
addpath(genpath( 'Functions' ) );

ANIMATE_ON = 1;

filename = 'v90_test';
par = getParameters_v0();

% Time vector.
t_vec = 0:0.01:par.t_end ;

q_start = get_q_start(); dq_start = zeros(numel(q_start),1) ;
state_start = [q_start; dq_start];

% % Dr. Ordonez: I am not positive of the actual
% % smallnumber that I had to choose but it was very small to get good performance.
% smallnumber = 1e-6; % v242.
% % smallnumber = 1e-10; % Up to v238.
% % Default RelTol = 1e-3. Default AbsTol = 1e-6.
% options = odeset('AbsTol',smallnumber,'RelTol',smallnumber, ...
%     'Stats','on', 'OutputFcn',@odeplot);

options = odeset('AbsTol',1e-6,'RelTol',1e-3, ...
    'Stats','on', 'OutputFcn',@odeplot);


runtime_start = tic;
q0 = findIC;
state_start = [q0(1); q0(2); q0(3); q0(4); q0(5); 0; 0; 0; 0; 0];

% % Dr. Ordonez: I am not positive of the actual
% % smallnumber that I had to choose but it was very small to get good performance.
% smallnumber = 1e-6; % v242.
% % smallnumber = 1e-10; % Up to v238.
% % Default RelTol = 1e-3. Default AbsTol = 1e-6.
% options = odeset('AbsTol',smallnumber,'RelTol',smallnumber, ...
%     'Stats','on', 'OutputFcn',@odeplot);

% options = odeset('AbsTol',1e-6,'RelTol',1e-3, ...
%     'Stats','on', 'OutputFcn',@odeplot);

options = odeset('AbsTol',1e-6,'RelTol',1e-3, ...
    'Stats','on');

runtime_start = tic;
t_store = [];
state_store = [];


for iter = 1:numel(t_vec)-1
    %     u= -0.1;
    q = state_start(1:numel(state_start)/2);
    dq = state_start(numel(state_start)/2+1:end);
    lb = [-100; -inf(9,1)];
    ub = [100; inf(9,1)];
    tic
    w_star = quadprog(Q_opt(q,dq),c_opt(q,dq),[],[],Aeq_opt(q,dq),beq_opt(q,dq),lb,ub);
    toc

    %     keyboard
    u = w_star(1);
    %     u = 0;
    dyn = @(t,state) dynamics_wrapper(t,state,u);

    if(iter == numel(t_vec)-1)
        tspan = [t_vec(iter) t_vec(iter+1)];
    else
        tspan = [t_vec(iter) t_vec(iter+1)-1e-10];
    end

    [t_temp, state_temp] = ode23( dyn, tspan, state_start, options );
    t_store = [t_store; t_temp];
    state_store = [state_store; state_temp];
    state_start = state_temp(end,:).';
    t_vec(iter)

end

t_temp = t_store;
state_temp = state_store;

runtime_end = toc(runtime_start);
disp( [ 'ODE solver computation time = ' num2str(runtime_end) 'seconds.'  ] )
t_vec_out = t_temp.'; state = state_temp.';
clear t_temp state_temp;


%% Save workspace for the plot and animation scripts.
save([filename, '.mat'])

%% Plot: run plot_*.m
%% Animate: run animate_*.m



FPS = 60;
t_interp = 0:1/FPS:max(t_vec_out);
x_interp = interp1(t_vec_out, state(1,:), t_interp);
z_interp = interp1(t_vec_out, state(2,:), t_interp);
theta_interp = interp1(t_vec_out, state(3,:), t_interp);


phi_plot = linspace(-pi,pi,1000);
p_plot = linspace(-10,10,5000);
roll_points = rollDrawFunc(phi_plot);
terr_points = terrDrawFunc(p_plot);


if(ANIMATE_ON)
    v = VideoWriter('roll_test.mp4','MPEG-4');
    v.FrameRate = FPS;
    open(v)

end

figure
% set(gcf,'outerposition',[992 352 1313 1071])
set(gcf,'outerposition',[6458    66    2456    1343],'color',[1 1 1])

for iter = 1:numel(t_interp)
    cla
    theta = theta_interp(iter);
    R = [cos(theta) sin(theta); -sin(theta) cos(theta)]; %
    roll_rotate = R*roll_points;
    hold on
    ref_line = [0 1; 0 0];
    rot_rel_line = R*ref_line;
    fill(roll_rotate(1,:)+x_interp(iter), roll_rotate(2,:)+z_interp(iter), [0.4 0.6 1],'linewidth',2)
    % plot(rot_rel_line(1,:)+x_interp(iter), rot_rel_line(2,:)+z_interp(iter), 'k--')
    plot(x_interp(1:iter), z_interp(1:iter), 'k-','markersize',8,'linewidth',2)
    plot(x_interp(iter), z_interp(iter), 'k.','markersize',15)
    plot(terr_points(1,:), terr_points(2,:), 'k-','linewidth',2)
%     plot(x_interp(iter), z_interp(iter), 'rx')
axis off
    axis equal
    axis([-1 3 -1 1])
    hold off
    drawnow

    if(ANIMATE_ON)
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    % pause(0.1)


%     if(mod(iter,60) == 1)
% 
%     pause
%     end

end
if(ANIMATE_ON)
    close(v);
end