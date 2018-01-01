clear all;
clc;
close all;

M = 1;                      % Mass of the cart in Kg (Ref: Exercise 3.18 and Figure 3.19)
m = 0.1;                    % Mass of the inverted pendulum in Kg (Ref: 3.18 and Figure 3.19)
g = 10 ;                    % Gravitational force is assumed to be 10m/sec^2 as given in Exercise 3.18
l =1; % m

A = [ 0 1 0 0;              % The A matrix which is calculated by hand (Refer report for more details)
    0 0 -1 0;
    0 0 0 1;
    0 0 11 0;];

B  = [0;1;0;-1];            % The B matrix which is calculated by hand (Refer report for more details)
C = [1 0 0 0];              % The C matrix which is calculated by hand (Refer report for more details)
D = [0];                    % The D matrix which is calculated by hand (Refer report for more details)
tspan = 0:.01:4;

Controllability_Flag = 0;   % The flag is set and the program for feedback continues only if the flag is set

EiganValue_Given = eig (A);
display = sprintf('The Eigan Values of the given Model are %f, %f, %f, %f', EiganValue_Given(1), EiganValue_Given(2), EiganValue_Given(3), EiganValue_Given(4));
disp(display);
Rnk = rank(ctrb(A,B));  % is it controllable

if(Rnk == size(A,1))
    Controllability_Flag = 1;                                           % The program can proceed further as the system is controllable
end

%-----------------------------------------------------------------------------------------------%
%-------------------Linear state feedback gain with state variables available-------------------%
%-----------------------------------------------------------------------------------------------%

if(Controllability_Flag == 1)
    
    P1 = [-5.1; -5.2; -5.3; -5.4];
    P2 = [-3.1; -3.2; -3.3; -3.4];
    P3 = [-1.1; -1.2; -1.3; -1.4];
    K1 = place(A,B,P1);                             % Linear state feedback gain equation for the 1st set of poles
    K2 = place(A,B,P2);                             % Linear state feedback gain equation for the 2nd set of poles
    K3 = place(A,B,P3);                             % Linear state feedback gain equation for the 3rd set of poles
    
    x0 = [0.1; 0; 0.1; 0];                          % initial state for the system
    xf = [0;0;0;0];                                 % Final State required for the system
    
    %----------------> for the P1 poles set <---------------%
    
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K1*(x)),tspan,x0);
    figure(1);
    
    display = sprintf('The Linear State feedback gain matrix [%f %f %f %f]', K1(1), K1(2), K1(3), K1(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    u = -K1*x';
    figure(20);
    plot(t,u);
    title('The control law graph for P1');
    
    %------------------------> End <------------------------%
    %----------------> for the P2 poles set <---------------%
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K2*(x)),tspan,x0);
    figure(2);
    
    display = sprintf('The Linear State feedback gain matrix [%f %f %f %f]', K2(1), K2(2), K2(3), K2(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set P2');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set P2');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set P2');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set P2');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    u = -K2*x';
    figure(21);
    plot(t,u);
    title('The control law graph for P2');
    
    %------------------------> End <------------------------%
    %----------------> for the P3 poles set <---------------%
    
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K3*(x)),tspan,x0);
    figure(3);
    
    display = sprintf('The Linear State feedback gain matrix [%f %f %f %f]', K3(1), K3(2), K3(3), K3(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set P3');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set P3');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set P3');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set P3');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    u = -K3*x';
    figure(22);
    plot(t,u);
    title('The control law graph for P3');
    
    %------------------------> End <------------------------%
end

% %-----------------------------------------------------------------------------------------------%
% %--------------------------------------------- End ---------------------------------------------%
% %-----------------------------------------------------------------------------------------------%
% %
% %-----------------------------------------------------------------------------------------------%
% %------------------------Combined state estimator and controller--------------------------------%
% %-----------------------------------------------------------------------------------------------%
if(Controllability_Flag == 1)
    
    P = [-20 -21 -22 -23];              % The estimator poles should be 4 - 6 times more than the poles of the linear state feedback
    L = place(A',C',P)';                % To get the value of L
    A_EstNew = [A-B*K1 -B*K1;zeros(size(A)) A-L*C];
    B_Est = [B;zeros(size(B))];
    C_Est = [C zeros(size(C))];
    D = 0;
    x0 = [0;0;0.1; 0;0 ;0 ;0.1; 0];
    
    [t,x] = ode45(@(t,x)diffEq_combined(x,A_EstNew),tspan,x0);
    
    
    figure(4);                              % Represents the state variables
    subplot(4,1,1);    plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    
    subplot(4,1,2);    plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    
    subplot(4,1,3);    plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    
    subplot(4,1,4);    plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    
    figure(5);                              % Represents the estimator error
    subplot(4,1,1);    plot(t,x(:,5));
    title('State Variables error X1 - X1~ for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    
    subplot(4,1,2);    plot(t,x(:,6));
    title('State Variables error X2 - X2~ for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    
    subplot(4,1,3);    plot(t,x(:,7));
    title('State Variables error X3 - X3~ for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    
    subplot(4,1,4);    plot(t,x(:,8));
    title('State Variables error X4 - X4~ for Pole set P1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
end
% %-----------------------------------------------------------------------------------------------%
% %--------------------------------------------- End ---------------------------------------------%
% %-----------------------------------------------------------------------------------------------%
%
% %-----------------------------------------------------------------------------------------------%
% %--------------------------------------------- LQR ---------------------------------------------%
% %-----------------------------------------------------------------------------------------------%

%-------------------------------------->LQR Cost = 1<--------------------------------------%

cost1 = 1;
cost2 = 1;
if(Controllability_Flag == 1)
    
    Q = [cost1 0 0 0;              % The ARE equation requires Q R in the Q
        0 0 0 0;                % which is taken distance and angle has
        0 0 cost2 0;               % been penalized
        0 0 0 0;];
    R = 1;
    K_lqr = lqr(A,B,Q,R);       % using Matlab inbuilt function to find the state feedback gain using LQR
    A_lqr = A-B*K_lqr;
    B_lqr = B;
    C_lqr = C;
    D_lqr = D;
    
    
    x0 = [0.1; 0; 0.1; 0];      % Initial values at start
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K_lqr*(x)),tspan,x0);
    figure(6)
    
    display = sprintf('The Linear State feedback gain matrix for LQR [%f %f %f %f]', K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set given by LQR cost = 1');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set given by LQR cost = 1');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set given by LQR cost = 1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set given by LQR cost = 1');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    u = -K_lqr*x';
    figure(24);
    plot(t,u);
    title('The control law graph for LQR 1');
    %--------------------------------------> End <--------------------------------------%
    %-------------------------------------->LQR Cost = 10<--------------------------------------%
    cost1 = 10;
    cost2 = 10;
    
    Q = [cost1 0 0 0;              % The ARE equation requires Q R in the Q
        0 0 0 0;                % which is taken distance and angle has
        0 0 cost2 0;               % been penalized
        0 0 0 0;];
    R = 1;
    K_lqr = lqr(A,B,Q,R);       % using Matlab inbuilt function to find the state feedback gain using LQR
    A_lqr = A-B*K_lqr;
    B_lqr = B;
    C_lqr = C;
    D_lqr = D;
    
    
    x0 = [0.1; 0; 0.1; 0];      % Initial values at start
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K_lqr*(x)),tspan,x0);
    figure(7)
    
    display = sprintf('The Linear State feedback gain matrix for LQR [%f %f %f %f]', K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set given by LQR cost = 10');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set given by LQR cost = 10');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set given by LQR cost = 10');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set given by LQR cost = 10');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    u = -K_lqr*x';
    figure(25);
    plot(t,u);
    title('The control law graph for LQR 2');
    
    %--------------------------------------> End <--------------------------------------%
    %-------------------------------------->LQR Cost = 100<--------------------------------------%
    
    cost1 = 100;
    cost2 = 100;
    
    Q = [cost1 0 0 0;              % The ARE equation requires Q R in the Q
        0 0 0 0;                % which is taken distance and angle has
        0 0 cost2 0;               % been penalized
        0 0 0 0;];
    R = 1;
    K_lqr = lqr(A,B,Q,R);       % using Matlab inbuilt function to find the state feedback gain using LQR
    A_lqr = A-B*K_lqr;
    B_lqr = B;
    C_lqr = C;
    D_lqr = D;
    
    
    x0 = [0.1; 0; 0.1; 0];      % Initial values at start
    [t,x] = ode45(@(t,x)diffEq(x,m,M,l,g,-K_lqr*(x)),tspan,x0);
    figure(8)
    
    display = sprintf('The Linear State feedback gain matrix for LQR [%f %f %f %f]', K_lqr(1), K_lqr(2), K_lqr(3), K_lqr(4));
    disp(display);
    
    subplot(4,1,1);     plot(t,x(:,1));
    title('State Variables X1 = X / Output variable for Pole set given by LQR cost = 100');
    xlabel('time in sec')                           % x-axis label
    ylabel('distance from origin point in metres')  % y-axis label
    
    subplot(4,1,2);     plot(t,x(:,2));
    title('State Variables X2 = Xdot for Pole set given by LQR cost = 100');
    xlabel('time in sec')                           % x-axis label
    ylabel('speed in meter/sec')                    % y-axis label
    
    subplot(4,1,3);     plot(t,x(:,3));
    title('State Variables X3 = theta for Pole set given by LQR cost = 100');
    xlabel('time in sec')                           % x-axis label
    ylabel('angle of the pendulum in radians')      % y-axis label
    
    subplot(4,1,4);     plot(t,x(:,4));
    title('State Variables X4 = theta dot for Pole set given by LQR cost = 100');
    xlabel('time in sec')                           % x-axis label
    ylabel('angular velocity in rads/sec')          % y-axis label
    
    u = -K_lqr*x';
    figure(26);
    plot(t,u);
    title('The control law graph for LQR 3');
    
    %--------------------------------------> End <--------------------------------------%
end
% %-----------------------------------------------------------------------------------------------%
% %--------------------------------------------- End ---------------------------------------------%
% %-----------------------------------------------------------------------------------------------%