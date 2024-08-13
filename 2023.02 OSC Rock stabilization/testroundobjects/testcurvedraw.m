clc
clear
close all

% Define test parametric functions

% px = @(p) cos(p)+0.1*cos(3*p);
% py = @(p) sin(p)+0.05*sin(6*(p-0.4));

px1 = @(p) cos(p)+0.1*cos(3*p);
py1 = @(p) 3/4*sin(p)+0.05*sin(6*(p-pi/8));

px2 = @(p) p;
py2 = @(p) sin(p)/5+cos(2*p)/5+sin(5*p)/10-p/10-1;

t = linspace(-pi,pi,1000);

x1 = px1(t);
y1 = py1(t);

x2 = px2(t);
y2 = py2(t);

plot(x1,y1,'k-', x2,y2,'k-',0,0,'r.')
axis equal