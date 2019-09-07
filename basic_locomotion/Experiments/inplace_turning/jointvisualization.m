clear all;
close all;

t1 = [0:0.02:1];
t2 = [0:-0.02:-1];

a1 = 0.2*sin(2*pi*1*t1+0)+0.05;
a2 = 0.2*sin(2*pi*1*t2+0)-0.05;

figure;
hold on;
grid on;
plot(t1, a1);
plot(t2, a2);

hold off;