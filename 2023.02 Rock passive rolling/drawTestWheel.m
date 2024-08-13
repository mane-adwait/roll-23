% drawTestWheel

clc
clear
close all

xw = @(phi) 0.5*cos(phi)+0.1*cos(2*(phi-pi/8));
zw = @(phi) -sin(phi)- 0.05*sin(3*(phi-pi/6));

phi = linspace(-pi,pi,1000);

plot(xw(phi),zw(phi),'k-')
axis equal