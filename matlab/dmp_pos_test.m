function dmp_pos_test()

set_matlab_utils_path();

%% Load training data

load('kuka_data.mat', 'Data');

Timed = Data.Time;
Pd_data = Data.Pos;
dPd_data = Data.Vel;
ddPd_data = Data.Accel;

%% Write data to binary format
% fid = fopen('train_data.bin','w');
% write_mat(Timed, fid, true);
% write_mat(Pd_data, fid, true);
% write_mat(dPd_data, fid, true);
% write_mat(ddPd_data, fid, true);
% fclose(fid);

Ts = Timed(2)-Timed(1);

%% initialize DMP
a_z = 20;
b_z = a_z/4;
train_method = DMP_orient.LWR;
can_clock_ptr = CanonicalClock();
shape_attr_gat_ptr = SigmoidGatingFunction(1.0, 0.5);
N_kernels = [60; 60; 60];
dmp_p = DMP_pos(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gat_ptr, DMP.DMP_STD);

disp('Position DMP training...')
tic
offline_train_mse = dmp_p.train(train_method, Timed, Pd_data, dPd_data, ddPd_data);
offline_train_mse
toc


%% DMP simulation
disp('DMP simulation...');
tic
P0 = Pd_data(:,1);
Pg = Pd_data(:,end);
T = Timed(end);
dt = Ts;
[Time, P_data, dP_data, ddP_data] = simulatePosDMP(dmp_p, P0, Pg, T, dt);
toc

%% Plot results

Pgd = Pd_data(:,end);
Pqd_data = zeros(3, size(Pd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = Pgd - Pd_data(:,j);
end

Pq_data = zeros(3, size(P_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = Pg - P_data(:,j);
end

line_width = 2.5;
 
figure('Position', [200 200 600 500]);
y_labels = {'$e_{q,x}$','$e_{q,y}$', '$e_{q,z}$'};
for i=1:3
   subplot(3,1,i);
   hold on;
   plot(Time, Pq_data(i,:), 'LineWidth', line_width);
   plot(Timed, Pqd_data(i,:), 'LineWidth', line_width, 'LineStyle','--');
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


end


