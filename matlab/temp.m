clc;
close all;
clear;

load('data/orient_data.mat', 'Data');

Timed = Data.Time;
Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;

n_data = length(Timed);
Pd_data = zeros(3, n_data);
Pd_data(1,:) = (0:(n_data-1))/n_data;

Q0d = Qd_data(:,1);
Qgd = Qd_data(:,end);
ks = 0.9;
e0 = ks*quatLog( quatProd( Qgd, quatInv(Q0d) ) );
Q0 = Q0d;
Qg = quatProd(quatExp(e0), Q0);

P_data = Pd_data;
Q_data = Qd_data;

% Q_rot = quatProd(Qg,quatInv(Qgd));
Q_rot = quatProd( quatProd(Qg,quatInv(Q0)) , quatInv( quatProd(Qgd,quatInv(Q0d)) ));
for j=1:n_data
   Q_data(:,j) = quatProd(Q_rot, Qd_data(:,j)); 
end

ax = plot_3Dpath_with_orientFrames(Pd_data, Qd_data, 'LineWidth',2.0, 'numberOfFrames',12, 'frameScale',0.03, 'frameLineWidth',2.0);
plot_3Dpath_with_orientFrames(P_data(:,1:4:end), Q_data(:,1:4:end), 'axes',ax, 'LineWidth',2.0, 'numberOfFrames',12, 'frameScale',0.03, 'frameLineWidth',2.0, ...
    'frameXAxisColor', [1 0 1], 'frameYAxisColor', [0 0.5 0], 'frameZAxisColor', [0 1 1], 'frameLineStyle','--', 'animated',false);

