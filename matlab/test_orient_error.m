clc;
close all;
clear;

a_z = 100;
b_z = a_z/4;

D = a_z;
K = a_z*b_z;

load('leonidas_data.mat','Data');

Time = Data.Time;
n_data = length(Time);
Ts = Time(2)-Time(1);
tau = Time(end);

Qd_data = Data.Quat;
vRotd_data = Data.RotVel;
dvRotd_data = Data.RotAccel;
Qd0 = Qd_data(:,1);
Qg = Qd_data(:,end);

Q_data = [];
vRot_data = [];
dvRot_data = [];
Q0 = Qd0;
Q = Q0;
vRot = zeros(3,1);
dvRot = zeros(3,1);

for i=1:n_data
   
    Q_data = [Q_data Q];
    vRot_data = [vRot_data vRot];
    dvRot_data = [dvRot_data dvRot];
    
    Qd = Qd_data(:,i);
    vRotd = vRotd_data(:,i);
    dvRotd = dvRotd_data(:,i);
    
    logQgQ = quatLog( quatProd( Qg, quatInv(Q) ) );
    logQgQd = quatLog( quatProd( Qg, quatInv(Qd) ) );
    dvRot = dvRotd - (D/tau)*(vRot - vRotd) - (K/tau^2)*(-logQgQ +logQgQd);
    
    Q = quatProd( quatExp(vRot*Ts), Q);
    vRot = vRot + dvRot*Ts;
    
end

qPos = zeros(3, n_data);
qPosd = zeros(3, n_data);
for i=1:size(qPos,2)
    qPos(:,i) = quatLog( quatProd( Q_data(:,i), quatInv(Qg) ) );
    qPosd(:,i) = quatLog( quatProd( Qd_data(:,i), quatInv(Qg) ) );
end

%% ================================================================
%% ================================================================

figQ = figure('Position',[250 250 1300 800]);
axQ = cell(3,3);
for i=1:3
    for j=1:3
        axQ{i,j} = subplot(3,3,(i-1)*3+j,'Parent',figQ);
        hold(axQ{i,j}, 'on');
    end
end


for i=1:3
    plot(Time, qPos(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{1,i});
    plot(Time, vRot_data(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{2,i});
    plot(Time, dvRot_data(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{3,i});
    
    plot(Time, qPosd(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{1,i});
    plot(Time, vRotd_data(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{2,i});
    plot(Time, dvRotd_data(i,:), 'LineWidth',1.5, 'LineStyle','-', 'Parent',axQ{3,i});
    
    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15, 'Parent',axQ{3,i}); end
end
title('$X$', 'interpreter','latex', 'fontsize',18, 'Parent',axQ{1,1});
title('$Y$', 'interpreter','latex', 'fontsize',18, 'Parent',axQ{1,2});
title('$Z$', 'interpreter','latex', 'fontsize',18, 'Parent',axQ{1,3});
ylabel('$log(Q*Q_g^{-1})$ [$rad$]', 'interpreter','latex', 'fontsize',16, 'Parent',axQ{1,1});
ylabel('$\omega$ [$rad/s$]', 'interpreter','latex', 'fontsize',16, 'Parent',axQ{2,1});
ylabel('$\dot{\omega}$ [$rad/s^2$]', 'interpreter','latex', 'fontsize',16, 'Parent',axQ{3,1});
