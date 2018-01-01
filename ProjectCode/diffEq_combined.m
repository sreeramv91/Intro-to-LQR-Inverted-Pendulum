function dx = diffEq_combined( x,A_EstNew,tspan,x0)
%Function definition:
%
% The function is to represent the differnetial equation combined state
% controller and estimator.
% The state variables of the system are given below:
% x1 = x    x2 = xdot      x3 = theta   x4 = theta dot
% The output variable is y = x1

dx = A_EstNew*x;

end

