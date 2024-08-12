% Convert matrix indices to row-wise linear index.

% Example: m = 3, n = 2.

% Input:
%   1,1     1,2
%   2,1     2,2
%   3,1     3,2

% Output:
%   1       2
%   3       4
%   5       6 


% Useful for the subplot command.

function lin_index = M2L_index_row(m, n, im, in)

completed_rows = (im - 1)*n ;
lin_index = completed_rows + in ;

end