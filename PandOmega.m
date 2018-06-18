% A Quasi-Infinite Horizon Nonlinear Model Predictive Control Scheme, Chen
% Allgower
% Simulation results, section 5, examples
% June 17, 2018, Zhuo, Uvic

%% This script can find the terminal penalty matrix P and the terminal region \Omega_{\alpha}

%% clear workspace
clear all
clc


P = [16.5926 11.5926;11.5926 16.5926];
kappa = 0.95;

Lphimax = kappa * min(eig(P))/norm(P,2)

mu = 1;
% find the terminal region


alpha = 0.1;


A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];



x0 = [0.01;0.01];


nonlcon = @quadconstraint;

[x_opt,Lphi] = fmincon(@(x)phix(x,mu),x0,A,b,Aeq,beq,lb,ub,nonlcon)





function [c,ceq] = quadconstraint(x)
    P = [16.5926 11.5926;11.5926 16.5926];
    c = x'*P*x - 0.1;%alpha;
    ceq = [];
end

function fxu = phix(x,mu)
    T = 0.1;
    K = [2.1180 2.1180];
    Ak = [-1.0590 -0.0590;-0.0590 -1.0590];
    
    %u = 2.118*x(1,1) + 2.118*x(2,1);
    u = K*x;
    y(1,1) = x(1,1) + T*(x(2,1) + u*(mu + (1 - mu)*x(1,1)));
    y(2,1) = x(2,1) + T*(x(1,1) + u*(mu - 4*(1 - mu)*x(2,1)));
    
    fx = y - Ak*x;
    
    fxu = norm(fx,2)/norm(x,2);
    fxu = -fxu;
end
