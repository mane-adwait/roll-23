clc
clear
close all

% Define test parametric functions

% px = @(p) cos(p)+0.1*cos(3*p);
% py = @(p) sin(p)+0.05*sin(6*(p-0.4));

px1 = @(p) cos(p)+0.2*cos(2*p);
py1 = @(p) -0.5*sin(p)-0.02*sin(3*p-pi/2);

px2 = @(p) p;
py2 = @(p) -p.^2;

t = linspace(-pi,pi,1000);

x1 = px1(t);
y1 = py1(t);

x2 = px2(t);
y2 = py2(t);

plot(x1,y1,'k-', x2,y2,'k-',0,0,'r.')
axis equal