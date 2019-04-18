function plotExecResults(sim_data_filename, train_data_filename)

clc;
close all;
clear;

addpath('utils/io_lib/');
addpath('utils/math_lib/');

if (nargin < 1), sim_data_filename = 'exec_data.bin'; end
if (nargin < 2), train_data_filename = 'kuka_train_data.bin'; end

sim_data_filename = ['../data/' sim_data_filename];
train_data_filename = ['../data/' train_data_filename];

%% ========  Read exec data  ============
fid = fopen(sim_data_filename,'r');
if (fid < 0), error(['Failed to open "' sim_data_filename '"']); end

a_z = read_scalar(fid, true, 'double');
Time = read_mat(fid, true);
x_data = read_mat(fid, true);
P_data = read_mat(fid, true);
Q_data = read_mat(fid, true);
P_robot_data = read_mat(fid, true);
Q_robot_data = read_mat(fid, true);
Fext_data = read_mat(fid, true);
fclose(fid);


%% ========  Read train data  ============
fid = fopen(train_data_filename,'r');
if (fid < 0), error(['Failed to open "' train_data_filename '"']); end

q0 = read_mat(fid, true);
Timed = read_mat(fid, true);
Pd_data = read_mat(fid, true);
dPd_data = read_mat(fid, true);
ddPd_data = read_mat(fid, true);
Qd_data = read_mat(fid, true);
vRotd_data = read_mat(fid, true);
dvRotd_data = read_mat(fid, true);
fclose(fid);


%% ========  Plot data  ============

Fext_norm = zeros(1,size(Fext_data,2));
for j=1:length(Fext_norm)
    Fext_norm(j) = norm(Fext_data(:,j));
end

Pgd = Pd_data(:,end);
Pd_err_data = zeros(3, size(Pd_data,2));
for j=1:size(Pd_err_data,2)
    Pd_err_data(:,j) = Pgd - Pd_data(:,j);
end

Pg = Pgd;
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

Qg = Qgd;
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
    plot(Time, Fext_norm, 'LineWidth',1.5, 'LineStyle','-', 'Color','cyan');
    ylabel('$F_{ext}$', 'interpreter','latex', 'fontsize',14);
    xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
   	legend({fext_labels{i},'$||\mathbf{f}_{ext}||$'}, 'interpreter','latex', 'fontsize',14);
    axis tight;
    hold off;
end


end