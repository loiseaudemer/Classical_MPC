% A Quasi-Infinite Horizon Nonlinear Model Predictive Control Scheme, Chen
% Allgower
% Simulation results, section 5, examples
% May 8, 2018, Zhuo, Uvic

%% clear workspace
clear all
clc

% Parameters
Q = [0.5 0;0 0.5];
R = 1;

% Jaccobian Linearization
A = [0 1;1 0];
B = [0.5;0.5];



% State feedback gain
[K,S,e] = lqr(A,B,Q,R,N);   %  return: u = -Kx, assuption u = Kx

Ak = A - B*K;

% eig(Ak)

% kappa is chosen as 0.95
kappa = 0.95;

% lyapunov equation: (Ak + kI)^T*P + P*(Ak + kI) + (Q + K^TRK) = 0;

Al = Ak + kappa*eye(2);
Ql = Q + K'*R*K;

% P, terminal penalty matrix
P = lyap(Al,Ql);

Lphimax = kappa * min(eig(P))/norm(P);


%% Nonlinear discrete model

x0 = [1,1]';
u0 = zeros(1,1);
mu = 0.5; % mu =1 linear, mu = 0 completely nonlinear
N = 10;

function [u_pred, x_pred] = solveOptCtrl(x0)
    u0 = 

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = -1;
    ub = 1;
    
    
    u_pred = fmincon(@(u_pred), costfun, u0, A, b,...
                Aeq, beq, lb, ub);
    

end




% discrete-time dynamics
function y = dynamics(x,u)
    y(1) = x(1) + T*(x(2) + u*(mu + (1 - mu)*x(1)));
    y(2) = x(2) + T*(x(1) + u*(mu - 4*(1 - mu)*x(2)));
end

% %% general form
% function cost = stagecost(x,u,t)
%     cost = x'*Q*x + u'*R*u;
% end
% 
% function cost = terminalcost(x,u,t)
%     cost = x'*P*x
% end
% 
% function cost = costfun(stagecost,terminalcost)
% 
% 
% end
function cost = costfun(x,u,t)
    cost = 0;
    for i = 0:N
        cost = cost + x'*Q*x + u'*R*u;
        x = dynamics(x,u);
        
    end
    cost = cost + x'*P*x;
end
    
