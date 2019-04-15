function plotSimResults(sim_data_filename, train_data_filename)

clc;
close all;
clear;

addpath('utils/io_lib/');
addpath('utils/math_lib/');

if (nargin < 1), sim_data_filename = 'sim_data.bin'; end
if (nargin < 2), train_data_filename = 'train_data.bin'; end

sim_data_filename = ['../data/' sim_data_filename];
train_data_filename = ['../data/' train_data_filename];

%% ========  Read sim data  ============
fid = fopen(sim_data_filename,'r');
if (fid < 0), error(['Failed to open "' sim_data_filename '"']); end

a_z = read_scalar(fid, true, 'double');
Time = read_mat(fid, true);
P_data = read_mat(fid, true);
dP_data = read_mat(fid, true);
ddP_data = read_mat(fid, true);
fclose(fid);

%% ========  Read sim data  ============
fid = fopen(train_data_filename,'r');
if (fid < 0), error(['Failed to open "' train_data_filename '"']); end
    
Timed = read_mat(fid, true);
Pd_data = read_mat(fid, true);
dPd_data = read_mat(fid, true);
ddPd_data = read_mat(fid, true);
fclose(fid);


%% ========  Plot data  ============


Pg = P_data(:,end);
Pq_data = zeros(3, size(P_data,2));
for j=1:size(Pq_data,2)
    Pq_data(:,j) = Pg - P_data(:,j);
end

Pgd = Pd_data(:,end);
Pqd_data = zeros(3, size(Pd_data,2));
for j=1:size(Pqd_data,2)
    Pqd_data(:,j) = Pgd - Pd_data(:,j);
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
   if (i==1), legend({'proposed DMP', 'demo'}, 'interpreter','latex', 'fontsize',16, 'Position',[0.7 0.78 0.27 0.15]); end
   if (i==1), title(['Position error: $e_p = P_g - P$, $\alpha_z = ' num2str(a_z) '$'], 'interpreter','latex', 'fontsize',18); end
   if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',17); end
   hold off;
end

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
   if (i==4), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15); end
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