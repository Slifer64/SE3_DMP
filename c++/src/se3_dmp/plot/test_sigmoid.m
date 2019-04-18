clc;
close;
clear;

% a_force: 2
% c_force: 5 # Newton
% a_pos: 700
% c_pos: 0.02 # meters
% a_orient: 4
% c_orient: 5 # rads
            
a = 2;
c = 5;

x = 0:(c/1000):2*c;

y = 1 ./ ( 1 + exp(a*(x-c)) );

figure;
plot(x, y, 'LineWidth',2.0);
xlabel('input');
ylabel('output');
title(['a = ' num2str(a) ', c = ' num2str(c)]);

