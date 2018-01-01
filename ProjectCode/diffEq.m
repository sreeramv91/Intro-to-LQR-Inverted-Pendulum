function dx = diffEq(x,m,M,l,g,u)

%Function definition:
%
% The function is to represent the differnetial equation of the linerized 
% system model. The linearization is done by hand refer the report.
% The state variables of the system are given below:
% x1 = x    x2 = xdot      x3 = theta   x4 = theta dot
% The output variable is y = x1

Sx = sin(x(3));
Cx = cos(x(3));

D = (M+m)- m*Cx^2;

dx(1,1) = x(2);                                         % x1 dot
dx(2,1) = (1/D)*(u + m*l*x(4)^2*Sx - m*g*Sx*Cx);        % x2 dot
dx(3,1) = x(4);                                         % x3 dot
dx(4,1) = (1/l*D)*(m*l*x(4)*Sx*Cx + (m+M)*g*Sx - u*Cx); % x4 dot