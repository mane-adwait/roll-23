function [p] = RoundRectangle(p1, p2, R, centering)


N = 100;

L = norm(p1-p2);
theta = atan2(p2(2)-p1(2),p2(1)-p1(1));
th_vec = linspace(-pi/2,pi/2,N);

rotmat = [cos(theta), -sin(theta); sin(theta), cos(theta)];

if(centering == 0)
    x = [L+R*cos(th_vec) 0-R*cos(th_vec) L];
    y = [R*sin(th_vec) R*sin(-th_vec) -R];
else
    x = [L/2+R*cos(th_vec) -L/2-R*cos(th_vec) L/2];
    y = [R*sin(th_vec) R*sin(-th_vec) -R];
end

p = rotmat*[x;y]+[p1(1);p1(2)]*ones(1,numel(x));

% plot(p(1,:),p(2,:),'k-')
% axis equal
