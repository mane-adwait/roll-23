%FULLDIFF Total derivative wrt time, not just partial derivatives from diff
%
%  fulldiff(function)
%  fulldiff(function,dep_vars)  % t is independant variable
%  fulldiff(function,{dvar1, dvar2})
%  fulldiff(function,dep_vars, num_dir)
%  fulldiff(function,num_dir)   % assumes dep_var is x (or closest) wrt t
%
%  function     - symbolic function to take derivative
%  variable     - single dependant variable, eg. x
%  {var1, var2} - time dep variables eg {x,y}
%  num_dir      - number of derivatives
%
%  Mandatory format.  All time dependant variables must be specified in
%  variables or {var1, var2, ...}; if none are provided the letter found in
%  function closest to 'x' will be used.  Any derivative in function must
%  start with the letter d or d2 etc. eg dx, d2x, d3x.  dx does not have to
%  be in dep_vars, if x is time dependant, then dx will be automatically.
%  Bug Fix: 03/26/05 - properly captures higher (>2) order derivatives
%  Bug Fix: 03/02/06 - fixed 3rd order derivative, thanks to Bruno G.
%  Bug Fix: 04/01/14 - fixed omission of similar vars, thanks Henrique L.S.
%
%  df     df dx   df dy
%  --  =  -- -- + -- --
%  dt     dx dt   dy dt
%
%  Example:
%  
%  syms x y dx d2y
%
%  f = x*y*dx^2*d2y
%  fulldiff(f) % (assumes x only) produces
%  %       3
%  %   y dx  d2y + 2 x y dx d2y d2x
%  % whereas
%  fulldiff(f,{x, y}) % produces
%  %       3                              2                2
%  %   y dx  d2y + 2 x y dx d2y d2x + x dx  d2y dy + x y dx  d3y
%
%  written by: Tim Jorris
%              updated: 1 May 2014
