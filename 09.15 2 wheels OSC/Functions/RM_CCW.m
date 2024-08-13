% Rotation matrix for a coordinate system with CCW +ve rotation.

% Y |
%   |         Z out of the page
%   |
%    ---- X
% 
% Positive angle corresponds to a counter-clockwise (CCW) rotation.

% 2023 September 13. Adwait Mane.

function out = RM_CCW(angle)

out = [ cos(angle), -sin(angle);
        sin(angle), cos(angle)] ;

end
