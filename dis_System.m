function [x] = dis_System(x0,u)
%discrete time system describtion
%x0 : initial state at time t0 of size 8x1
%u  : input signal for T time steps of size 4x7T

    [~,T] = size(u);
    x(1) = x0;
    A;
    B;
    for k = 1:T
        x(k+1) = A*x(k)+B*u;
    end

end

