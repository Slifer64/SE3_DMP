function dmp_pos_orient_stopping_test2()

set_matlab_utils_path();

%% Load training data

load('data/leonidas_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_orient.LWR;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
N_kernels = [60; 60; 60];
dmp_p = DMP_pos(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr, DMP.DMP_STD);
dmp_o = DMP_orient(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr);

disp('Position DMP training...')
tic
offline_train_mse_p = dmp_p.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
offline_train_mse_p
toc

disp('Orient DMP training...')
tic
offline_train_mse_o = dmp_o.train(train_method, Timed, Qd_data, vRotd_data, dvRotd_data);
offline_train_mse_o
toc


%% DMP simulation
disp('DMP simulation...');
tic
P0 = Pd_data(:,1);
Pg = Pd_data(:,end);
Q0 = Qd_data(:,1);
Qg = quatExp(1.0*quatLog(Qd_data(:,end)));
T = Timed(end);
dt = Ts;

%% set initial values
can_clock_ptr = dmp_p.can_clock_ptr;

t = 0.0;

x = 0.0;
dx = 0.0;

P = P0;
dP = zeros(3,1);
Z = dP;
ddP = zeros(3,1);
Q = Q0;
vRot = zeros(3,1);
dvRot = zeros(3,1);
phi = vRot;

P_robot = P;
dP_robot = dP;
ddP_robot = ddP;
Q_robot = Q;
vRot_robot = vRot;
dvRot_robot = dvRot;

P_f = zeros(3,1);
dP_f = zeros(3,1);
ddP_f = zeros(3,1);
Q_f = quatProd( Q_robot, quatInv(Q));
vRot_f = zeros(3,1);
dvRot_f = zeros(3,1);

Fext = zeros(6,1);

t_end = T;
tau0 = t_end;
can_clock_ptr.setTau(tau0);

iters = 0;
Time = [];
P_data = [];
dP_data = [];
ddP_data = [];
Q_data = [];
vRot_data = [];
dvRot_data = [];
x_data = [];

P_robot_data = [];
dP_robot_data = [];
ddP_robot_data = [];
Q_robot_data = [];
vRot_robot_data = [];
dvRot_robot_data = [];

Fext_data = [];

eq_f = zeros(3,1);
eq_f_data = [];

Q_f_data = [];

% === Cartesian spring-damper params ===
Mp = [4; 4; 4];
Kp = [50; 50; 50];
Dp = 2*sqrt(Mp.*Kp);

% === Orientation spring-damper params ===
Mo = [1; 1; 1];
Ko = [10; 10; 10];
Do = 2*sqrt(Mo.*Ko);

a_force = 2;
c_force = 5; % Newton
a_pos = 800;
c_pos = 0.02; % meters
a_orient = 4;
c_orient = 5; % rads

sigmoid = @(a,c,x) 1 ./ ( 1 + exp(a*(x-c)) );
sigmoid_dot = @(a,c,x) -a*exp(a*(x-c)) ./ ( 1 + exp(a*(x-c)) ).^2;

getExternalWrench = @(t) 1*[-6*exp(-0.5*((t-2.2)/0.5).^2); -4.5*exp(-0.5*((t-2.5)/0.5).^2); 5*exp(-0.5*((t-5.8)/0.5).^2); ...
                          5.7*exp(-0.5*((t-2.7)/0.5).^2); 0.7*exp(-0.5*((t-6.1)/0.5).^2); 0.7*exp(-0.5*((t-6.4)/0.5).^2)];

%% simulate
while (true)
    
    % ddP_robot = ddP + ddP_f;
    % dvRot_robot = dvRot + dvRot_f;
    
    %% data logging
    Time = [Time t];
    P_data = [P_data P];
    dP_data = [dP_data dP];  
    ddP_data = [ddP_data ddP];
    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];  
    dvRot_data = [dvRot_data dvRot];
    x_data = [x_data x];
    
    P_robot_data = [P_robot_data P_robot];
    dP_robot_data = [dP_robot_data dP_robot];
    ddP_robot_data = [ddP_robot_data ddP_robot];
    Q_robot_data = [Q_robot_data Q_robot];
    vRot_robot_data = [vRot_robot_data vRot_robot];
    dvRot_robot_data = [dvRot_robot_data dvRot_robot];
    
    Fext_data = [Fext_data Fext];
    
    eq_f_data = [eq_f_data eq_f];
    Q_f_data = [Q_f_data Q_f];
    
    %% admittance
    Fext = getExternalWrench(t);
    ddP_robot = ddP + ( Fext(1:3) - Dp.*(dP_robot-dP) - Kp.*(P_robot-P) ) ./ Mp;
    eq_f = quatLog(quatProd(Q_robot, quatInv(Q)));
    dvRot_robot = dvRot + ( Fext(4:6) - Do.*(vRot_robot - vRot) - Ko.*eq_f ) ./ Mo;

    %% apply stopping
    s_f = sigmoid(a_force, c_force, norm(Fext));
    s_p = sigmoid(a_pos, c_pos, norm(P_f));
    s_q = sigmoid(a_orient, c_orient, norm(eq_f)*180/pi);
    s_tau = s_f*s_p*s_q;

    %% Update phase variable
    can_clock_ptr.setTau(tau0/s_tau);
    dx = can_clock_ptr.getPhaseDot(x);
    tau = can_clock_ptr.getTau();
    
    ds_f = sigmoid_dot(a_force, c_force, x)*dx;
    ds_p = sigmoid_dot(a_pos, c_pos, x)*dx;
    ds_q = sigmoid_dot(a_orient, c_orient, x)*dx;
    ds_tau = ds_f*s_p*s_q + s_f*ds_p*s_q + s_f*s_p*ds_q;
    tau_dot = -ds_tau*tau0/s_tau^2;
    
    %% DMP simulation
    dmp_p.calcStatesDot(x, P, Z, P0, Pg);
    dZ = dmp_p.getDz();
    dP_prev = dP;
    dP = dmp_p.getDy();
    ddP2 = dZ / tau;
    % ddP = (dP - dP_prev)/dt;
    ddP = (dZ - tau_dot * dP) / tau;
    
    abs(ddP - ddP2)
    pause
    
    dmp_o.calcStatesDot(x, Q, phi, Q0, Qg);
    dphi = dmp_o.getDphi();
    vRot_prev = vRot;
    vRot = dmp_o.getOmega();
    % dvRot = dphi / tau;
    % dvRot = (vRot - vRot_prev)/dt;
    dvRot = (dphi - tau_dot * vRot) / tau;

    %% Stopping criteria
    if (t>=t_end) % && norm(y-g)<5e-3 && norm(dy)<5e-3)
        break;
    end

    %% Numerical integration
    iters = iters + 1;
    t = t + dt;
    x = x + dx*dt;
    P = P + dP*dt;
    Z = Z + dZ*dt;
    %dP = dP + ddP*dt;
    Q = quatProd( quatExp(vRot*dt), Q);
    phi = phi + dphi*dt;
    %vRot = vRot + dvRot*dt;
    
    P_robot = P_robot + dP_robot*dt;
    dP_robot = dP_robot + ddP_robot*dt;
    Q_robot = quatProd( quatExp(vRot_robot*dt), Q_robot);
    vRot_robot = vRot_robot + dvRot_robot*dt;
    % dP_robot = dP + dP_f;
    % vRot_robot = vRot + vRot_f;
    
    P_f = P_robot - P; 
    dP_f = dP_robot - dP;
    ddP_f = ddP_robot - ddP;
    Q_f = quatProd( Q_robot, quatInv(Q));
    vRot_f = vRot_robot - vRot;
    dvRot_f = dvRot_robot - dvRot;

end

toc

eq_f_hat_data = zeros(size(eq_f_data));
for j=1:size(eq_f_hat_data,2), eq_f_hat_data(:,j) = quatLog(Q_f_data(:,j)); end
   
% figure
% plot(eq_f_hat_data')
% 
% figure
% plot(eq_f_data')

Fext_norm = zeros(1,size(Fext_data,2));
for j=1:length(Fext_norm)
    Fext_norm(j) = norm(Fext_data(:,j));
end

Pgd = Pd_data(:,end);
Pd_err_data = zeros(3, size(Pd_data,2));
for j=1:size(Pd_err_data,2)
    Pd_err_data(:,j) = Pgd - Pd_data(:,j);
end

P_err_data = zeros(3, size(P_data,2));
for j=1:size(P_err_data,2)
    P_err_data(:,j) = Pg - P_data(:,j);
end

P_robot_err_data = zeros(3, size(P_robot_data,2));
for j=1:size(P_robot_err_data,2)
    P_robot_err_data(:,j) = Pg - P_robot_data(:,j);
end

Qgd = Qd_data(:,end);
Qd_err_data = zeros(3, size(Qd_data,2));
for j=1:size(Qd_err_data,2)
    Qd_err_data(:,j) = quatLog( quatProd(Qgd, quatInv(Qd_data(:,j))) );
end

Q_err_data = zeros(3, size(Q_data,2));
for j=1:size(Q_err_data,2)
    Q_err_data(:,j) = quatLog( quatProd(Qg, quatInv(Q_data(:,j))) );
end

Q_robot_err_data = zeros(3, size(Q_robot_data,2));
for j=1:size(Q_robot_err_data,2)
    Q_robot_err_data(:,j) = quatLog( quatProd(Qg, quatInv(Q_robot_data(:,j))) );
end

Y_err = [P_err_data; Q_err_data];
Y_robot_err = [P_robot_err_data; Q_robot_err_data];
Yd_err = [Pd_err_data; Qd_err_data];

fext_labels = {'$f_x$','$f_y$','$f_z$','$\tau_x$','$\tau_y$','$\tau_z$'};

for i=1:size(Y_err,1)
    figure;
    subplot(3,1,1);
    plot(Time, x_data, 'LineWidth',2.5);
    ylabel('phase variable', 'interpreter','latex', 'fontsize',14);
   	legend({'$x$'}, 'interpreter','latex', 'fontsize',14);
    axis tight;
    subplot(3,1,2);
    hold on;
    plot(Time, Y_err(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color','blue');
    plot(Time, Y_robot_err(i,:), 'LineWidth',2.5, 'LineStyle','-', 'Color',[0.85, 0.33, 0.1]);
    plot(Timed, Yd_err(i,:), 'LineWidth',2.5, 'LineStyle',':', 'Color','green');
    ylabel('Y', 'interpreter','latex', 'fontsize',14);
   	legend({'DMP','robot','demo'}, 'interpreter','latex', 'fontsize',14);
    axis tight;
    hold off;
    subplot(3,1,3);
    hold on;
    plot(Time, abs(Fext_data(i,:)), 'LineWidth',2.5, 'Color','red');
    plot(Time, Fext_norm, 'LineWidth',2.5, 'LineStyle','--', 'Color',[0.75 0 0.75]);
    ylabel('$F_{ext}$', 'interpreter','latex', 'fontsize',14);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
   	legend({fext_labels{i},'$||\mathbf{f}_{ext}||$'}, 'interpreter','latex', 'fontsize',14);
    axis tight;
    hold off;
end

return;

%% ======  Plot position results  =======

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, P_err_data(i,:), 'LineWidth', line_width);
   plot(Timed, Pd_err_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'pos DMP', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Position error: $e_p = P_g - P$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
hold on;
plot3(P_data(1,:), P_data(2,:), P_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pd_data(1,:), Pd_data(2,:), Pd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
hold off;

figure;
Q_labels = {'$x$','$y$', '$z$'};
Qd_labels = {'$x_d$','$y_d$', '$z_d$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, P_data(i,:), 'LineWidth', line_width);
   plot(Timed, Pd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({Q_labels{i}, Qd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Position', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
vRot_labels = {'$p_x$','$p_y$', '$p_z$'};
vRotd_labels = {'$p_{d,x}$','$p_{d,y}$', '$p_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, dP_data(i,:), 'LineWidth', line_width);
   plot(Timed, dPd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({vRot_labels{i}, vRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Velocity', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
dvRot_labels = {'$\dot{p}_x$','$\dot{p}_y$', '$\dot{p}_z$'};
dvRotd_labels = {'$\dot{p}_{d,x}$','$\dot{p}_{d,y}$', '$\dot{p}_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, ddP_data(i,:), 'LineWidth', line_width);
   plot(Timed, ddPd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({dvRot_labels{i}, dvRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Cartesian Acceleration', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

%% ======  Plot Orientation results  =======

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Q_err_data(i,:), 'LineWidth', line_width);
   plot(Timed, Qd_err_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
   ylabel(y_labels{i}, 'interpreter','latex', 'fontsize',20);
   axis tight;
   if (i==1), legend({'proposed DMP', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Quaternion error: $e_q = log(Q_g * Q^{-1})$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

figure;
hold on;
plot3(Pq_data(1,:), Pq_data(2,:), Pq_data(3,:), 'LineWidth', line_width, 'LineStyle','-');
plot3(Pqd_data(1,:), Pqd_data(2,:), Pqd_data(3,:), 'LineWidth', line_width, 'LineStyle','--');
hold off;

figure;
Q_labels = {'$\eta$','$\epsilon_1$', '$\epsilon_2$', '$\epsilon_3$'};
Qd_labels = {'$\eta_d$','$\epsilon_{d,1}$', '$\epsilon_{d,2}$', '$\epsilon_{d,3}$'};
for i=1:4
   subplot(4,1,i);
   hold on;
   plot(Time, Q_data(i,:), 'LineWidth', line_width);
   plot(Timed, Qd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({Q_labels{i}, Qd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Unit Quaternion', 'interpreter','latex', 'fontsize',17); end
   if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
vRot_labels = {'$\omega_x$','$\omega_y$', '$\omega_z$'};
vRotd_labels = {'$\omega_{d,x}$','$\omega_{d,y}$', '$\omega_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, vRot_data(i,:), 'LineWidth', line_width);
   plot(Timed, vRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({vRot_labels{i}, vRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Rotational Velocity', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

figure;
dvRot_labels = {'$\dot{\omega}_x$','$\dot{\omega}_y$', '$\dot{\omega}_z$'};
dvRot2_labels = {'old $\dot{\omega}_x$','old $\dot{\omega}_y$', 'old $\dot{\omega}_z$'};
dvRotd_labels = {'$\dot{\omega}_{d,x}$','$\dot{\omega}_{d,y}$', '$\dot{\omega}_{d,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, dvRot_data(i,:), 'LineWidth', line_width);
   plot(Timed, dvRotd_data(i,:), 'LineWidth', line_width, 'LineStyle',':');
   legend({dvRot_labels{i}, dvRotd_labels{i}}, 'interpreter','latex', 'fontsize',15);
   if (i==1), title('Rotational Acceleration', 'interpreter','latex', 'fontsize',17); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
   hold off;
end

end


