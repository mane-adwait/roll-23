% Friday 2022 Oct 27

clear all; close all;
addpath('../Functions')
load('../v5.mat')


theta = state(3,:) ;
phi = state(4,:) ;
p = state(5,:) ;
mu = theta + phi - p ;

% Create the figure and set the properties:
fig500 = figure(500); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure 
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion]; 

% Set the figure position using the syntax [left bottom width height].
cFig.Units = 'normalized'; cFig.OuterPosition = [0.5 0.5 0.49 0.49];
% movegui(cFig,'northeast'); % Another way to specify figure location.

cFig.Color = 'white'; 


draw.m = 1; draw.n = 1;

for im = 1:draw.m
    for in = 1:draw.n

        numPlots = 4; % Select the number of plots on each subplot.

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


subplot(draw.m,draw.n,1); 
title(filename, ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, theta, ...
    t_vec_out, phi, ...
    t_vec_out, p, ...
    t_vec_out, mu, ...
    'LineWidth', 1.5); grid on;
% ylabel('m','Interpreter','latex','FontSize',20);
legend({    '$\theta$',...
    '$\phi$',...
    '$p$',...
    '$ \theta + \phi - p $',...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

xlabel('Time (s)', 'Interpreter','latex','FontSize',20);



return

%%
[term1, term2, A_sigma1] = calc_terms(state) ;

% Create the figure and set the properties:
fig500 = figure(500); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure 
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion]; 

% Set the figure position using the syntax [left bottom width height].
cFig.Units = 'normalized'; cFig.OuterPosition = [0.5 0.5 0.49 0.49];
% movegui(cFig,'northeast'); % Another way to specify figure location.

cFig.Color = 'white'; 


draw.m = 1; draw.n = 1;

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

subplot(draw.m,draw.n,1); 
title(filename, ...
    'Interpreter','latex','FontSize',18);
plot(   t_vec_out, term1(1,:), ...
    t_vec_out, term2(1,:), ...
    t_vec_out, A_sigma1(1,:), ...
    'LineWidth', 1.5); grid on;
% ylabel('m','Interpreter','latex','FontSize',20);
legend({    'term1',...
    'term2',...
    '$\sigma_{1}$',...
    },'Location','eastoutside',...
    'Interpreter','latex','FontSize',20);

xlabel('Time (s)', 'Interpreter','latex','FontSize',20);

% figure; plot(t_vec_out, A_sigma1); title(sigma1) ;
% 
% figure; plot(t_vec_out, state(5,:)); 


% load(['C:\Code\l3harris-high-mobility\Motion planning\2022 Wheel on a ...' ...
%     'parabola sim\2022.09.22 v210/Results/v210/v210.mat'])

% load(['C:/Code/l3harris-high-mobility/Motion planning/2022 Wheel on a ...' ...
%     'parabola sim/2022.09.22 v210/Results/v210/v210.mat'])

