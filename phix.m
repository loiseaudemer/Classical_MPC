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

