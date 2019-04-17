function [Time, P_data, dP_data, ddP_data] = simulatePosDMP(dmp_p, P0, Pg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
can_clock_ptr = dmp_p.can_clock_ptr;

t = 0.0;

x = 0.0;
dx = 0.0;

P = P0;
dP = zeros(3,1);
ddP = zeros(3,1);

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
x_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    % x_data = [x_data x];

    %% DMP simulation
    z_c = zeros(3,1);
    ddP = dmp_p.getAccel(x, P, dP, P0, Pg, z_c);

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    if (t>=t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    dP = dP + ddP*dt;

end


end

