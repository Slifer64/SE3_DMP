close all;

load data/fotis_data.mat Qd_data Ts

log_data = zeros(3, size(Qd_data, 2));


% logQData = zeros(3, length(QData));
% dlogQData = zeros(3, length(QData));
% ddlogQData = zeros(3, length(QData));
% 
% omegaData = zeros(3, length(QData));
% domegaData = zeros(3, length(QData));

for i = 1:size(Qd_data, 2)
    
   log_data(:, i) = quatLog(Qd_data(:, i)); 
    
end

[QData, omegaData, domegaData] = processQData(Qd_data, Ts, 0, 0.03);
[logQData, dlogQData, ddlogQData] = processData(log_data, Ts, 0, 0.03);


dQData = zeros(4, length(QData));
ddQData = zeros(4, length(QData));
ddQDataFromLog = zeros(4, length(QData));
domegaDataFromLog = zeros(3, length(QData));


for i = 1:size(Qd_data, 2)

    dQData(:, i) = 0.5 * quatProd([0; omegaData(:, i)], QData(:, i));
    
end

for i = 1:size(Qd_data, 2)
    prod1 = quatProd([0; domegaData(:, i)], QData(:, i));
    prod2 = quatProd([0; omegaData(:, i)], dQData(:, i));
    ddQData(:, i) = 0.5 * (prod1 + prod2);    
end

for i = 1:size(Qd_data, 2)
   
    J = getLogQToQJacobian( logQData(:, i) );
    Jdot = getJacobianAcceleration(logQData(:, i), dlogQData(:, i));
    ddQDataFromLog(:, i) = Jdot * dlogQData(:, i) + J * ddlogQData(:, i);
    
end

for i = 2:size(Qd_data, 2)
   
    prod1 = quatProd([0; omegaData(:, i)], dQData(:, i));
    tempdOmega = quatProd((ddQDataFromLog(:, i) - prod1), quatInv(QData(:, i)));
    domegaDataFromLog(:, i) = tempdOmega(2:4);
    
end

t = Ts:Ts:size(QData, 2) * Ts;

figure;
for i = 1:4
    subplot(4, 1, i);
    plot(t, ddQData(i, :), t, ddQDataFromLog(i, :));
end

figure;
for i = 1:3
    subplot(3, 1, i);
    plot(t, domegaData(i, :), t, domegaDataFromLog(i, :));
end