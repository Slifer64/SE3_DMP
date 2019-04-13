function [Time, Q_data, vRot_data, dvRot_data] = simulateOrientDMP(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
can_clock_ptr = dmp_o.can_clock_ptr;

t = 0.0;

x = 0.0;
dx = 0.0;

Q = Q0;
vRot = zeros(3,1);
dvRot = zeros(3,1);

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
Q_data = [];
vRot_data = [];
dvRot_data = [];
x_data = [];

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];  
    dvRot_data = [dvRot_data dvRot];
    % x_data = [x_data x];

    %% DMP simulation
    y_c = zeros(3,1);
    dvRot = dmp_o.getStatesDot(x, Q, vRot, Q0, Qg, y_c);

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
    Q = quatProd( quatExp(vRot*dt), Q);
    vRot = vRot + dvRot*dt;

end


end

