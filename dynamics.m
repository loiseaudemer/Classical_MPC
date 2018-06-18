% discrete-time dynamics
function y = dynamics(x,u)
    T = 0.1;
    mu = 0.5; % mu =1 linear, mu = 0 completely nonlinear
    y(1,1) = x(1,1) + T*(x(2,1) + u*(mu + (1 - mu)*x(1,1)));
    y(2,1) = x(2,1) + T*(x(1,1) + u*(mu - 4*(1 - mu)*x(2,1)));
end
