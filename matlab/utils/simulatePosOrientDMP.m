function [Time, P_data, dP_data, ddP_data, Q_data, vRot_data, dvRot_data] = simulatePosOrientDMP(dmp_p, dmp_o, P0, Q0, Pg, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
can_clock_ptr = dmp_p.can_clock_ptr;

t = 0.0;

x = 0.0;
dx = 0.0;

P = P0;
dP = zeros(3,1);
ddP = zeros(3,1);
Q = Q0;
vRot = zeros(3,1);
dvRot = zeros(3,1);

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
Q_data = [];
vRot_data = [];
dvRot_data = [];
x_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];  
    dvRot_data = [dvRot_data dvRot];
    % x_data = [x_data x];

    %% DMP simulation
    Zc_p = zeros(3,1);
    ddP = dmp_p.getAccel(x, P, dP, P0, Pg, Zc_p);
    Zc_o = zeros(3,1);
    dvRot = dmp_o.getRotAccel(x, Q, vRot, Q0, Qg, Zc_o);

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
    Q = quatProd( quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;

end


end

