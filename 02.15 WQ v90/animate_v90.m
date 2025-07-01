% Before running:
%   Select the appropriate .mat file to load.


%% Animate.
% Figure numbers 100 to 149.

clear all; close all;
addpath(genpath( 'Functions' ) );

% Need to manually specify which data file to read.
% load( 'Results/v_/v_.mat' )
load('v90.mat')

% Camera position options:
% "wheel": follows the wheel. Recommended for parabolic.
% "terrain": fixed to the terrain. Recommended for quartic.
cam_position = "wheel";

%% Create the figure and set the properties:

fig100 = figure(100); % For convenience, use figN = figure(N)
cFig = gcf; % This makes the following commands modular i.e. the figure
% handle does not need to be updated.
% cFig.Name = [fileName, fileVersion];

% Set the figure position using the syntax [left bottom width height].
% cFig.Units = 'normalized'; cFig.OuterPosition = [0.66 0.3 0.2 0.6]; % For parabolic terrain.
cFig.Units = 'normalized'; cFig.OuterPosition = [0.3 0.3 0.4 0.4];
% movegui(cFig,'northeast'); % Another way to specify figure location.

cFig.Color = 'white';

%% Setup.
[p_min, p_max] = bounds(state(5,:));
[x_min, x_max] = bounds(state(1,:));
[z_min, z_max] = bounds(state(2,:));

pvec = p_min-2:0.05:p_max+2 ;
terr = terrain(pvec) ;

% Plot the terrain.
terr_plot = plot(terr(1,:), terr(2,:), 'LineWidth', 1.5, 'Color','black');
hold on;     axis equal;    grid off;

%% Set the axes limits here if the camera is fixed to the terrain.
if cam_position == "terrain"
    cAx = gca;
    bb_tc = 3; % bounding box parameter for terrain camera.
    cAx.XLim = [x_min - par.r1*bb_tc, x_max + par.r1*bb_tc];
    cAx.YLim = [z_min - par.r1*bb_tc, z_max + par.r1*bb_tc]; % For parabolic terrain.
elseif cam_position == "wheel"
    % Set inside the animation loop.
else
    disp('Error: check camera position option.')
end


%% Plot frame by frame.
for k = 1:numel(t_vec)

    x_k = state(1,k); z_k = state(2,k); theta_k = state(3,k);

    % Plot the wheel.
    rim_x_k = x_k + par.r1*cos(theta_k); rim_z_k = z_k - par.r1*sin(theta_k);
    wheel_marker = plot([x_k rim_x_k],[z_k rim_z_k], 'LineWidth', 1.5, 'Color','blue');
    wheel = draw_circle([x_k, z_k], par.r1, 'blue');
    % wheel = viscircles([x_k z_k], par.r1, 'Color', 'blue');

    % Set the axes limits here if the camera follows the wheel.
    if cam_position == "terrain"
        % Set before animation loop.
    elseif cam_position == "wheel"
        cAx = gca;
        bb_wc = 7; % bounding box parameter for wheel camera.
        cAx.XLim = [x_k - par.r1*bb_wc, x_k + par.r1*bb_wc];
        cAx.YLim = [z_k - par.r1*bb_wc, z_k + par.r1*bb_wc];
    else
        disp('Error: check camera position option.')
    end

    % Add a time stamp.
    annBoxDim = [.41 .6 .3 .3]; % [left bottom width height].
    annString = ['t = ' num2str(t_vec_out(k),3) ' s' ];
    tStamp = annotation('textbox', annBoxDim, 'String', annString, 'FitBoxToText','on');

    videoFrames(k) = getframe(gcf);

    delete(wheel_marker); delete(wheel);
    delete(tStamp);
end

% Create and save a video file.
% Use 'MPEG-4' video profile if supported. Alternative: 'Motion JPEG AVI'
videoObj = VideoWriter(filename, 'MPEG-4');
videoObj.Quality = 10;
FPS = 10; videoObj.FrameRate = FPS;
open(videoObj);
writeVideo(videoObj, videoFrames);
videoObj.close();

function h = draw_circle(center, radius, color)
    % DRAW_CIRCLE Draws a circle with specified radius and color
    %
    % Inputs:
    %   center - [x, y] coordinates of the center of the circle
    %   radius - radius of the circle
    %   color  - color of the circle outline
    %
    % Example usage:
    %   wheel = draw_circle([x_k, z_k], par.r1, 'blue')

    theta = linspace(0, 2*pi, 100); % 100 points to make a smooth circle
    x = center(1) + radius * cos(theta);
    y = center(2) + radius * sin(theta);
    
    h = plot(x, y, 'Color', color, 'LineWidth', 1.5);
end
