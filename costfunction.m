
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