function [P,flag,q1,q2,q3,q4] = ik_func(pos,psi,a,d)
% Inverse kinematics function module
% specifications
a3 = a(3);
a4 = a(4);
d1 = d(1);
d5 = d(5);
px = pos(1);
py = pos(2);
pz = pos(3);

% theta 1
q1 = atan2d_05pi(py,px); % q1 = atand(py/px)
% fill T05: 4 by 4 matrix elements
nx = cosd(q1)*cosd(psi);
ny = sind(q1)*cosd(psi);
nz = sind(psi);
ax = -cosd(q1)*sind(psi);
ay = -sind(q1)*sind(psi);
az = -cosd(psi);
ox = -sind(q1);
oy = cosd(q1);
oz = 0;
P = [nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1];
P = del_nega_zeros(P);
% disp('========================================')
% disp('Given condition')
% disp(P)

% theta 2
m = (px-ax*d5)/cosd(q1);
n = pz - az*d5-d1;
e = (m^2+n^2+a3^2-a4^2)/2/a3;
isOutofWorkspace = n^2+m^2-e^2 < 0;
if isOutofWorkspace
    flag = NaN;
    return
end
q2 = [atand_0pi(e/sqrt(n^2+m^2-e^2))-atand_0pi(m/n),...
    atand_0pi(-e/sqrt(n^2+m^2-e^2))-atand_0pi(m/n)];

k1 = (m-a3*cosd(q2))/a4;
k2 = (n-a3*sind(q2))/a4;
% theta 3
q3 = atan2d(k2,k1) - q2;
% theta 4
q4 = psi - q2 - q3;
flag = 1;
end