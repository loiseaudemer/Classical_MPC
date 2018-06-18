clear all
clc

n = 2; % number of states
m = 1; % number of inputs
% Control scheme parameters
N = 30; % prediction horizon
mpciteration = 50;


treal = 0; % real time count, real system
x0 = [-0.523;0.744]; % initial state, measured current state
u0 = zeros(m,1); % initial input, initial guess for fmincon, solve optimal u
u0_opt = zeros(m,N); % predicted optimal input sequence



A = [];
b = [];
Aeq = [];
beq = [];
lb = -2;
ub = 2;

xm = x0;
xPlot(:,1) = xm;
for mpciter = 1:mpciteration
    u_pred_init = uShiftHori(u0,u0_opt,N);
    u0_opt = fmincon(@(u)costfunction(xm,u,N),u_pred_init,A,b,Aeq,beq,lb,ub);
    J(mpciter) = costfunction(xm,u0_opt,N);
    u_applied = u0_opt(:,1);
    % plot variables
    uPlot(mpciter) = u_applied;
    xPlot(:,mpciter+1) = xm;
    xm = dynamics(xm,u_applied);
end

figure(1)
k = 0:(mpciteration);
stairs(k(1:mpciteration),uPlot)

figure(2)
plot(k(1:mpciteration),J)

figure(3)
x1Traj = xPlot(1,:);
x2Traj = xPlot(2,:);
plot(x1Traj,x2Traj)
axis([-1 1 -1 1])





%% Subroutines
% discrete-time dynamics
function y = dynamics(x,u)
    T = 0.1;
    mu = 0.5; % mu =1 linear, mu = 0 completely nonlinear
    y(1,1) = x(1,1) + T*(x(2,1) + u*(mu + (1 - mu)*x(1,1)));
    y(2,1) = x(2,1) + T*(x(1,1) + u*(mu - 4*(1 - mu)*x(2,1)));
end


function u_pred_init = uShiftHori(u0,u_opt,N)
    m = length(u0);
    u_pred_init = zeros(m,N);
    u_pred_init(:,N) = u0;
    u_pred_init(:,[1:N-1]) = u_opt(:,[2:N]);
end


function cost = costfunction(xm,u_pred_init,N)
    % x0 is measured current state, sized by nx1...
    % u0 is the initial guess of optimal input, mx1
    n = length(xm);
    m = length(u_pred_init(:,1));
    
    % cost function parameters
    Q = [0.5 0;0 0.5];
    R = 1;
    P = [16.5926,11.5926;11.5926,16.5926];
    %P = eye(2);
    
    % initialize predicted state matrix and input matrix
    x_pred = zeros(n,(N+1));
    x_pred(:,1) = xm;
    u_pred = u_pred_init;
    
    %%  guess predicted input
    tvir = 0; % virtual time, used for future prediction, t = 0 means current time
    %u_pred(:,tvir+1) = u_pred_init(:,1); % should be modified 
    %x_pred(:,tvir+1) = x0; % the first colunm of x_pred is the real state!
    
    % initialize the cost value
    cost = 0;
    % compute the cost at current time instance with initial guess of input
    cost = cost + xm'*Q*xm + (u_pred(:,1))'*R*u_pred(:,1); % costsum at time = 0
    
    %tvir = tvir + 1; % tvir = 1;
    
    % stage cost
    % compute predicted state of next time instance
    for tvir = 1:(N-1)
        x_pred(:,(tvir+1)) =  dynamics(x_pred(:,tvir),u_pred(:,tvir));% tvir predicted state
        cost = cost + (x_pred(:,tvir+1))'*Q*x_pred(:,tvir+1) + (u_pred(:,tvir+1))'*R*u_pred(:,tvir+1); % cost sum of 0,1
    end
    % terminal cost
    x_pred(:,N+1) = dynamics(x_pred(:,N),u_pred(:,N));
    cost = cost + (x_pred(:,N+1))'*P*x_pred(:,N+1);
    
end