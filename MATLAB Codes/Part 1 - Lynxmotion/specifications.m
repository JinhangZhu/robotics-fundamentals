%%  DH convention specifications
d1 = 0.1;
a3 = 0.1;
a4 = 0.1;
d5 = 0.1;
alpha = [0 90 0 0 -90]';
a = [0 0 a3 a4 0]';
d = [d1 0 0 0 d5]';
q5 =0;

%%  Range of motions per axis
q1_range = [-90,90];
q2_range = [0,180];
q3_range = [-135,45];
q4_range = [-180,0];

%%  Animation specifications
xlim_range = [0 0.3];
ylim_range = [-0.2 0.2];
zlim_range = [0 0.2];
axes_limit = [xlim_range;ylim_range;zlim_range];

%%  Trajectory: 5 viapoints
viapoints = [0.2 0.1 0.1;0.2 0.05 0.15;0.2 0 0.1;0.2 -0.05 0.15;0.2 -0.1 0.1]';

save('specifications');