function [Time, Q_data, rotVel_data, rotAccel_data] = simulateDMPeo(dmp_o, Q0, Qg, T, dt)
%% Simulates a dmp encoding Cartesian orientation usning unit quaternions.


%% set initial values
can_clock_ptr = dmp_o.can_clock_ptr;

t = 0.0;

x = 0.0;
dx = 0.0;

Q = Q0;
rotVel = zeros(3,1);
rotAccel = zeros(3,1);

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
Q_data = [];
rotVel_data = [];
rotAccel_data = [];
x_data = [];
eo_data = [];

eo_0 = quatLog(quatProd(Qg, quatInv(Q0)));
eo = eo_0;
deo = zeros(3,1);
ddeo = zeros(3,1);
y = eo;
y0 = eo_0;
z = zeros(3,1);
dy = zeros(3,1);
dz = zeros(3,1);

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Q_data = [Q_data Q];
    rotVel_data = [rotVel_data rotVel];  
    rotAccel_data = [rotAccel_data rotAccel];
    % x_data = [x_data x];
    eo_data = [eo_data eo];

    %% DMP simulation
    % z_c = zeros(3,1);
    % rotAccel = dmp_o.getRotAccel(x, Q, rotVel, Q0, Qg, z_c);
    dmp_o.calcStatesDot(x, y, z, y0);
    
    dy = dmp_o.getDy();
    dz = dmp_o.getDz();

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria   
    if (t>1.5*t_end)
        warning('Time limit reached... Stopping simulation!');
        break;
    end
    
    if (t>=t_end && norm(eo)<0.02)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    y = y + dy*dt;
    z = z + dz*dt;
    
    rotVel_prev = rotVel;
    
    eo = eo + deo*dt;
    deo = dy;
   
    Q_e = quatProd(Qg, quatInv(Q));
    rotVel = DMP_eo.deo2rotVel(deo, Q_e);
    rotAccel = DMP_eo.ddeo2rotAccel(deo, rotVel, Q_e);
    
    Q = quatProd( quatExp(rotVel*dt), Q);
    rotVel = rotVel + rotAccel*dt;
    
%     Q = quatProd( quatInv(quatExp(eo)), Qg);
%     rotVel = DMP_eo.deo2RotVel(deo, quatProd(Qg, quatInv(Q)));
%     rotAccel = (rotVel - rotVel_prev) / dt;
    
    
    
end


end

