
clc; clear; close all;

mu = 5;
sigma = 0.5;
A = 1.5;

p = linspace(-6, 15, 1000);
x = p ;
y = A*exp(-0.5*((p-mu)/sigma).^2);


plot(x,y,'k-')

axis equal
grid on
% [xmin xmax ymin ymax]
axis([-6 15 -1 4])