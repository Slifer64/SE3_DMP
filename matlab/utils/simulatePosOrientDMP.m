function [Time, Y_data, dY_data, ddY_data, Q_data] = simulatePosOrientDMP(dmp, Y0, Yg, T, dt)
%% Simulates a dmp
% @param[in] dmp: Dim x 1 cell array, where each cell is a 1D DMP.
% @param[in] y0: Dim x 1 vector with the initial position..
% @param[in] g: Dim x 1 vector with the goal/target position.
% @param[in] T: Movement total time duration.
% @param[in] dt: Simulation timestep.
% @param[out] Time: 1 x N rowvector with simulation output timestamps.
% @param[out] Y_data: Dim x N matrix with simulation output positions.
% @param[out] dY_data: Dim x N matrix with simulation output velocities.
% @param[out] ddY_data: Dim x N matrix with simulation output accelerations.
%


%% set initial values
Pg = Yg(1:3);
Qg = Yg(4:7);

P0 = Y0(1:3);
Q0 = Y0(4:7);

y0 = zeros(6,1);
y0(1:3) = P0;
y0(4:6) = -quatLog( quatProd( Qg, quatInv(Q0) ) );

yg = zeros(6,1);
yg(1:3) = Pg;
yg(4:6) = [0 0 0]'; % -quatLog( quatProd( Qg, quatInv(Qg) ) );

can_clock_ptr = dmp{1}.can_clock_ptr;
Dim = length(dmp);
x = 0.0;
dx = 0.0;
ddy = zeros(Dim,1);
dy = zeros(Dim,1);
y = y0;
t = 0.0;
dz = zeros(Dim,1);
z = zeros(Dim,1);


Q = Q0;

t_end = T;
can_clock_ptr.setTau(t_end);

iters = 0;
Time = [];
Y_data = [];
dY_data = [];
ddY_data = [];
x_data = [];

Q_data = [];

for i=1:Dim
    dmp{i}.setY0(y0(i));
    dmp{i}.setG(yg(i));
end

%% simulate
while (true)

    %% data logging
    Time = [Time t];
    Y_data = [Y_data y];
    dY_data = [dY_data dy];  
    ddY_data = [ddY_data ddy];
    % x_data = [x_data x];
    
    Q_data = [Q_data Q];

    %% DMP simulation
    for i=1:Dim      
        
%         y_c = 0.0;
%         z_c = 0.0;
%         [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), yg(i), y_c, z_c);
%         ddy(i) = dz(i)/dmp{i}.getTau();
        
        dmp{i}.update(DMP.X,x, DMP.Y,y(i), DMP.Z,z(i), DMP.Zc,0, DMP.Yc,0);
        dy(i) = dmp{i}.getDy();
        dz(i) = dmp{i}.getDz();
        ddy(i) = dz(i)/dmp{i}.getTau();
    end

    ddpos = ddy(1:3);
    dv_rot = ddy(4:6);

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
    
    y(1:3) = y(1:3) + dy(1:3)*dt;
    
    v_rot =  dy(4:6);
    Q = quatProd( quatExp(v_rot*dt), Q);
    y(4:6) = -quatLog( quatProd(Qg, quatInv(Q) ) );
    
    z = z + dz*dt;

end


end

