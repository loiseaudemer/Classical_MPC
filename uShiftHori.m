function u_pred_init = uShiftHori(u0,u_opt,N)
    m = length(u0);
    u_pred_init = zeros(m,N);
    u_pred_init(:,N) = u0;
    u_pred_init(:,[1:N-1]) = u_opt(:,[2:N]);
end

