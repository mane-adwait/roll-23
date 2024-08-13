% Rotation matrix for the default Simscape coordinate system.

% Z |
%   |         Y into the page
%   |
%    ---- X
% 
% Positive angle corresponds to a clockwise rotation.

% 2022 August 01. Adwait Mane.

function out = RM(angle)

out = [ cos(angle), sin(angle);
        -sin(angle), cos(angle)] ;

end
