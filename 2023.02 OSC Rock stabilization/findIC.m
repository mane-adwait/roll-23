function q0 = findIC()

N = 1000;

% x = 0.3;
% z = -0.5272;
x = 2.4;
z = -0.3665;

p_limits = [-3 3]+0;
p_witness = linspace(p_limits(1), p_limits(2), N);
terr_witness = terrDrawFunc(p_witness);

phi_witness = linspace(-pi,pi,N);
roll_witness = rollDrawFunc(phi_witness);
roll_witness(1,:) = roll_witness(1,:)+x;
roll_witness(2,:) = roll_witness(2,:)+z;



R = @(theta) [cos(theta) -sin(theta); sin(theta) cos(theta)];


x_diff = ones(N,1)*terr_witness(1,:) - roll_witness(1,:).'*ones(1,N);
z_diff = ones(N,1)*terr_witness(2,:) - roll_witness(2,:).'*ones(1,N);
dist = x_diff.^2+z_diff.^2;
min_dist = min(min(dist));

index = find(dist == min_dist);
index = index(1);

i1 = ceil(index/N);
i2 = mod(index,N);



terr_point = [terr_witness(1,i1); terr_witness(2,i1)];
roll_point = [roll_witness(1,i2); roll_witness(2,i2)];

% x = 0;
% z = -min_dist;
phi = atan2(z-terr_point(2), terr_point(1)-x);
p = terr_point(1);

% q0 = [x; z; 0; phi; p];

q0 = [x; z; 0; phi_witness(i2); p]

clf
hold on
plot(terr_witness(1,:),terr_witness(2,:),'k-')
plot(terr_point(1),terr_point(2),'rx')
plot(roll_witness(1,:),roll_witness(2,:),'k-')
plot(roll_point(1),roll_point(2),'rx')
plot([x x+1*cos(phi)],[z z-1*sin(phi)],'k--')
axis equal
hold off

% keyboard
