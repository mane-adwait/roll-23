% 06Dec2020 Adwait Mane

% https://www.mathworks.com/matlabcentral/answers/449426-how-do-i-save-every-nth-frame-of-a-video-and-save-these-images-to-a-folder-of-my-choosing?s_tid=srchtitle


%% User input
readObj = VideoReader('StepClimb.mp4');
numFramesOut = 9; % choose such that an m X n grid is completely filled.
outNamePrefix = 'StepClimb ';

%% 
numFrames = readObj.NumFrames;
disp( ['numFrames = ' num2str(numFrames) ...
    '.      numFramesOut = ' num2str(numFramesOut) '.' ] );

%% Select output frames.

% frames = [1 5 20];
% frames = [5];

frames = round( linspace(1,numFrames,numFramesOut) );
% y = linspace(x1,x2,n) generates n points. The spacing between the points is
% (x2-x1)/(n-1). linspace is similar to the colon operator, ?:?, but gives 
% direct control over the number of points and always includes the endpoints. 
% Note that the frame interval varies slightly because of rounding.

% Alternate, if frame interval needs to be constant.
% frameInterval = floor(numFrames/numFramesOut);
% disp( ['frameInterval = ' num2str(frameInterval) ] );
% frames = 1:frameInterval:numFrames;

%% Save selected frames.

disp('Continue execution if OK. Otherwise adjust parameters and re-run.');
keyboard

for iFrame = 1:numel(frames)
  currentFrame = read(readObj, frames(iFrame));
  % outname = sprintf('Outname-%03d.png',frames(iFrame));
  outName = [outNamePrefix num2str(frames(iFrame)) '.png' ];
%   imwrite(currentFrame,outName);
end

% imshow(currentFrame); % plot current frame.

print(currentFrame)