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

N = 0;

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

%fun = @(x)([x(2) + u*(0.5+0.5*x(1));x(1) + u*(0.5 - 2*x(2)] - Ak*[x(1);x(2)];

% 
% %% Find the terminal region
% mu = 0.5; % linearity const
% 
% alpha = 10;
% cvx_begin
%     variables x1(1);
%     variables x2(1);
%     x = [x1;x2];
%     u = K(1,1)*x1 + K(1,2)*x2;
%     %fxkx = [x2 + u*0.5 + 0.5*u*x1;x1 + u*0.5 - 2*u*x2)];
%     fxkx = [u*x1;u*x2];
%     Akx = Ak*x;
%     phix = fxkx - Akx;
%     Lphi = norm(phix)/norm(x);
%     maximize Lphi;
%     subject to
%         x'*P*x <= alpha;
% cvx_end
