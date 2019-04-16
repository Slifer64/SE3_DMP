clc;
close;
clear;

a = 4;
c = 5;

x = 0:(c/1000):2*c;

y = 1 ./ ( 1 + exp(a*(x-c)) );

figure;
plot(x, y, 'LineWidth',2.0);
xlabel('input');
ylabel('output');
title(['a = ' num2str(a) ', c = ' num2str(c)]);

