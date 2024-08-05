% Before running:
%   Select the appropriate .mat file to load.
%   Select the appropriate get_u_aug().

clear all; close all;

addpath(genpath( 'auto' ) );
addpath(genpath( 'Functions' ) );
load('Results/v0/v0.mat')
% load('v122.mat')
% load('v130.mat')

nq = numel(q_start);
% q = state(1:nq); dq = state(nq+1:end);

for k = 1:numel(t_vec_out)

    M_aug = get_M_aug(state(:,k));
    f_aug = get_f_aug(state(:,k));

    nq_aug = numel(f_aug);

    u_aug(1:nq_aug,k) = get_u_aug_0(t_vec_out(k), nq_aug );

    d2q_aug = M_aug^-1 * (f_aug + u_aug(:,k));
%     d2q_aug = lsqr(M_aug, f_aug + u_aug(:,k), 1e-3, 40);

    d2q = d2q_aug(1:nq);
    n_lambda = numel(d2q_aug) - nq;
    lambda(1:n_lambda,k) = d2q_aug(nq+1:end);

end

%% Generate plots.
% Figure numbers 50 to 99.

% Create the figure and set the properties:
fig5 = figure(50); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure 
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion]; 

% Set the figure position using the syntax [left bottom width height].
cFig.Units = 'normalized'; cFig.OuterPosition = [0.5 0.05 0.5 0.95];
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

draw.m = 4; draw.n = 1; % Select the subplot grid size.

%% Set the axes properties.

for im = 1:draw.m
    for in = 1:draw.n

        numPlots = 4; % Select the number of plots on each subplot.

        subplot(draw.m,draw.n,im); % Select the subplot axes.
        % Note: update the last argument if plotting more than one column.

        % Set the axes properties
        cAx = gca;
        lineColors = linspecer(numPlots);
        axis on; cAx.TickLabelInterpreter = 'latex';
        cAx.ColorOrder = lineColors; cAx.FontSize = 20;
        hold on;

    end
end

%% Plot.

subplot(draw.m,draw.n,1); 
title( ['Input torque (' filename ')'], ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, u_aug(3,:), ...
    'LineWidth', 1.5); grid on;
ylabel('N.m.','Interpreter','latex','FontSize',20);
legend({    'u',...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);


subplot(draw.m,draw.n,2); % lam 1,3
title('Lagrange multipliers', ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, lambda(1,:), ...
    t_vec_out, lambda(3,:), ...
    'LineWidth', 1.5); grid on;
% ylabel('$\lambda$','Interpreter','latex','FontSize',20);
legend({    '$\lambda_{1}$', '$\lambda_{3}$'...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,3); % lam2
plot(   t_vec_out, lambda(2,:), ...
    'LineWidth', 1.5); grid on;
% ylabel('$\lambda$','Interpreter','latex','FontSize',20);
legend({    '$\lambda_{2}$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

subplot(draw.m,draw.n,4); % lam4
plot(   t_vec_out, lambda(4,:), ...
    'LineWidth', 1.5); grid on;
% ylabel('$\lambda$','Interpreter','latex','FontSize',20);
cAx = gca; % Current axes.
% cAx.YLim = [9.5, 10.1]; cAx.YTick = [9.6, 9.81, 10];
legend({    '$\lambda_{4}$' ...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

xlabel('Time (s)', 'Interpreter','latex','FontSize',20);

% %% Plot lambda 2, time <= 0.8 s, k = start:8.
% figure(51); cFig = gcf; movegui(cFig,'northwest');
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


