% Before running:
%   Select the appropriate .mat file to load.
%   Select the appropriate get_u_aug().

clear all; close all;

addpath(genpath( 'auto' ) );
addpath(genpath( 'Functions' ) );
load('Results/v0/v0.mat')
% load('v122.mat')
% load('v124.mat')

nq = numel(q_start);
% q = state(1:nq); dq = state(nq+1:end);


%% Generate plots.
% Figure numbers 1 to 49.

% Create the figure and set the properties:
fig1 = figure(1); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure 
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion]; 

% Set the figure position using the syntax [left bottom width height].
cFig.Units = 'normalized'; cFig.OuterPosition = [0 0.04 1 0.96];
% movegui(cFig,'northeast'); % Another way to specify figure location.

cFig.Color = 'white'; 

% Create the axes

% -------------------------------------------------------------------------
% ax1_fig5 = axes; % Create a single axes object for the figure.

% subplot(m,n,p) creates an m-by-n grid. p=1 is first column of the first
% row. p=2 is the second column of the first row, and so on.
%   1   2
%   3   4
%   5   6 ...

% -------------------------------------------------------------------------

draw.m = 5; draw.n = 2; % Select the subplot grid size.

%% Set the axes properties.

for im = 1:draw.m
    for in = 1:draw.n

        numPlots = 3; % Select the number of plots on each subplot.

%         Wrote my own function since sub2ind uses column-wise numbering.
        lin_index = M2L_index_row(draw.m,draw.n, im,in) ;
        subplot(draw.m,draw.n,lin_index); % Select the subplot axes.
        

        % Set the axes properties
        cAx = gca;
        lineColors = linspecer(numPlots);
        axis on; cAx.TickLabelInterpreter = 'latex';
        cAx.ColorOrder = lineColors; cAx.FontSize = 20;
        hold on;

    end
end

%% Plot q on the left side subplots.

subplot(draw.m,draw.n,1); 
title(['Positions (' filename ')'], ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, state(1,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m','Interpreter','latex','FontSize',20);
legend({    '$x$',...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);
% title('Input torque ', ...
%     'Interpreter','latex','FontSize',18);


subplot(draw.m,draw.n,3);
plot(   t_vec_out, state(2,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m','Interpreter','latex','FontSize',20);
legend({    '$z$', ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);


subplot(draw.m,draw.n,5); 
plot(   t_vec_out, state(3,:), ...
    'LineWidth', 1.5); grid on;
ylabel('rad','Interpreter','latex','FontSize',20);
legend({    '$\theta$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,7); 
plot(   t_vec_out, state(4,:), ...
    'LineWidth', 1.5); grid on;
ylabel('rad','Interpreter','latex','FontSize',20);
% cAx = gca; % Current axes.
% cAx.YLim = [9.5, 10.1]; cAx.YTick = [9.6, 9.81, 10];
legend({    '$\phi$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,9); 
plot(   t_vec_out, state(5,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m','Interpreter','latex','FontSize',20);
legend({    '$p$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

xlabel('Time (s)', 'Interpreter','latex','FontSize',20);

%% Plot dq on the right side subplots.

subplot(draw.m,draw.n,2); 
title(['Velocities (' filename ')'], ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, state(6,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m/s','Interpreter','latex','FontSize',20);
legend({    '$\dot{x}$',...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);
% title('Input torque ', ...
%     'Interpreter','latex','FontSize',18);


subplot(draw.m,draw.n,4);
plot(   t_vec_out, state(7,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m/s','Interpreter','latex','FontSize',20);
legend({    '$\dot{z}$', ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);


subplot(draw.m,draw.n,6); 
plot(   t_vec_out, state(8,:), ...
    'LineWidth', 1.5); grid on;
ylabel('rad/s','Interpreter','latex','FontSize',20);
legend({    '$\dot{\theta}$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,8); 
plot(   t_vec_out, state(9,:), ...
    'LineWidth', 1.5); grid on;
ylabel('rad/s','Interpreter','latex','FontSize',20);
% cAx = gca; % Current axes.
% cAx.YLim = [9.5, 10.1]; cAx.YTick = [9.6, 9.81, 10];
legend({    '$\dot{\phi}$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,10); 
plot(   t_vec_out, state(10,:), ...
    'LineWidth', 1.5); grid on;
ylabel('m/s','Interpreter','latex','FontSize',20);
legend({    '$\dot{p}$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

xlabel('Time (s)', 'Interpreter','latex','FontSize',20);


% %% Plot lambda 2, time <= 0.8 s, k = start:8.
% figure(10); cFig = gcf; movegui(cFig,'northwest');
% 
% % Set the axes properties
% cAx = gca;
% lineColors = linspecer(numPlots);
% axis on; cAx.TickLabelInterpreter = 'latex';
% cAx.ColorOrder = lineColors; cAx.FontSize = 20;
% hold on;
% 
% plot(t_vec_out(1:8), lambda(2,1:8), 'LineWidth', 1.5); 
% legend({    '$\lambda_{2}$' ...
%     },'Location','eastoutside',...
%     'Interpreter','latex','FontSize',20);
% xlabel('Time (s)', 'Interpreter','latex','FontSize',20);


