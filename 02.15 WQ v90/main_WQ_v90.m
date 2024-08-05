% See reamde_v*.txt for full instructions.

close all; clear all; 
addpath(genpath( 'Functions' ) );

filename = 'v90';
par = getParameters_v0();

% Time vector.
t_vec = 0:0.1:par.t_end ;

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
[t_temp, state_temp] = ode23( @dynamics_wrapper, t_vec, state_start, options );
runtime_end = toc(runtime_start); 
disp( [ 'ODE solver computation time = ' num2str(runtime_end) 'seconds.'  ] )
t_vec_out = t_temp.'; state = state_temp.'; 
clear t_temp state_temp;


%% Save workspace for the plot and animation scripts.
save([filename, '.mat'])

%% Plot: run plot_*.m
%% Animate: run animate_*.m
