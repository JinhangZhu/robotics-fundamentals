function [obstacles,obs_1,obs_2,obs_3] = createobstacles(PT_START,PT_GOAL)
%createobstacles Create obstacles in the plane
%   Scale the 10*10 plane to given-points-formed rectangle
n = 100;
obstacles = zeros(2,15*n-15);

%
width = abs(PT_GOAL(1)-PT_START(1));
height = abs(PT_GOAL(2)-PT_START(2));
left_bottom = [min(PT_START(1),PT_GOAL(1));min(PT_START(2),PT_GOAL(2))];

% Obstacle 1: blocking triangle
side_1 = [linspace(0.7,2.5,n);linspace(2.1,1.3,n)];
side_2 = [linspace(2.5,1.8,n);linspace(1.3,3.2,n)];
side_3 = [linspace(1.8,0.7,n);linspace(3.2,2.1,n)];
obs_1 = [side_1(:,1:n-1) ...
    side_2(:,1:n-1) ...
    side_3(:,1:n-1)];
    
% Obstacle 2: passed pentagon
side_4 = [linspace(5.0,5.2,n);linspace(3.0,1.9,n)];
side_5 = [linspace(5.2,6.9,n);linspace(1.9,1.8,n)];
side_6 = [linspace(6.9,7.1,n);linspace(1.8,3.1,n)];
side_7 = [linspace(7.1,6.3,n);linspace(3.1,4.1,n)];
side_8 = [linspace(6.3,5.0,n);linspace(4.1,3.0,n)];
obs_2 = [side_4(:,1:n-1) ...
    side_5(:,1:n-1) ...
    side_6(:,1:n-1)...
    side_7(:,1:n-1) ...
    side_8(:,1:n-1)];
    
% Obstacle 3: blocking polygon
side_9 = [linspace(5.0,6.3,n);linspace(7.2,7.4,n)];
side_10 = [linspace(6.3,7.9,n);linspace(7.4,6.0,n)];
side_11 = [linspace(7.9,9.0,n);linspace(6.0,6.5,n)];
side_12 = [linspace(9.0,9.0,n);linspace(6.5,7.6,n)];
side_13 = [linspace(9.0,6.8,n);linspace(7.6,9.3,n)];
side_14 = [linspace(6.8,4.5,n);linspace(9.3,9.1,n)];
side_15 = [linspace(4.5,5.0,n);linspace(9.1,7.2,n)];
obs_3 = [side_9(:,1:n-1) ...
    side_10(:,1:n-1) ...
    side_11(:,1:n-1)...
    side_12(:,1:n-1)...
    side_13(:,1:n-1)...
    side_14(:,1:n-1)...
    side_15(:,1:n-1)];  

obs_1(1,:) = obs_1(1,:)/10*width+left_bottom(1);
obs_1(2,:) = obs_1(2,:)/10*height+left_bottom(2);
obs_2(1,:) = obs_2(1,:)/10*width+left_bottom(1);
obs_2(2,:) = obs_2(2,:)/10*height+left_bottom(2);
obs_3(1,:) = obs_3(1,:)/10*width+left_bottom(1);
obs_3(2,:) = obs_3(2,:)/10*height+left_bottom(2);
obstacles = [obs_1 obs_2 obs_3];


end

